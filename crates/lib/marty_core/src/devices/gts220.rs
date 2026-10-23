/*
    devices::gts220 — GTS-220 (HITEL-II) custom board devices:
    fixed-mode planar video + NVSRAM window (paged FS storage).

    Ported from the RE-verified Python model in emu/gts220_boot.py /
    emu/gts220_live.py (hook_vram_write/hook_vram_read, vga_write, render_rgb).

    Video hardware summary (see GTS-220/docs/ADDRESSMAP.md):
      - VRAM: 0xA0000, 4 planes x 64KB, VGA-style latched writes
      - Fixed 640x550 @ 16 colors, 80-byte stride, MSB first
      - Color pipeline (RE 2026-10-19): pixel → ATC color-plane enable →
        ATC palette → DAC (256 x 6-bit RGB, firmware-programmed) → RGB.
        The firmware loads a standard-VGA-like palette at boot and
        re-programs it at runtime.
      - CRTC data writes: the firmware programs all 25 CRTC registers once
        at boot from a mode table (video_init 0xE4074); runtime CRTC writes
        never occur (scroll is software). Registers 0x0C/0x0D are tracked.
      - Port 0x3DA: bit3 toggles on each read (vsync), bit0 always set (DE),
        read resets the ATC index/data flip-flop (real VGA behavior, used
        by the firmware before ATC programming).
      - All other 0x3B0-0x3DF port reads return 0x00
      - No hardware text mode: glyph rendering is done in software by the
        firmware reading the external font ROM (0x20000-0x9FFFF linear
        window, chip = bus & 0x7FFFF) and writing VRAM pixels.

    NVSRAM window (bus 0xB0000-0xBFFFF, port 0x3E2 = page latch):
      - Storage: KM681000B 128KB battery-backed SRAM = 2 x 64KB banks.
        "Data ROM" is a retired name from the flash hypothesis (2026-10-14);
        the chip dump proves the font mask ROM holds no FS metadata, and the
        KM681000B datasheet fixes the medium at 128KB = exactly 2 banks.
      - Page 0 = canonical bank (FAT chain @0xFE00, dir @0xF200)
        Page 1 = mirror bank (chain @FE00/F000, dir @E400/F200)
        The FULL 64KB window is banked: the dir mirror at page1@F200 shares
        its offset with the page0 canonical, so E000-FFFF is banked too
        (the old "split" model is retired). This is what makes the 3-way
        checksum vote meaningful (canonical vs 2 mirrors).
      - Firmware writes only 0/1 to the latch (42 call sites audited:
        27x arg=0, 15x arg=1, zero >=2). Latch values >=2 have no physical
        bank; the window content is left unchanged (floating).
      - Font mask ROM chip dump check: 0xF200/0xFE00 hold glyph bit
        patterns, not FS metadata — the canonical lives only in NVSRAM.

    Known divergence from the Python model: map_mask 0 is treated as 0x0F
    (all planes) exactly as in the Python model (`mm = seq[2] or 0x0F`).
*/

use crate::{
    bus::{
        BusInterface, DeviceRunTimeUnit, IoDevice, MemRangeDescriptor, MemoryMappedDevice,
        NO_IO_BYTE,
    },
    cpu_common::LogicAnalyzer,
    device_traits::videocard::{
        BufferSelect, ClockingMode, CursorInfo, DisplayAperture, DisplayApertureDesc,
        DisplayApertureType, DisplayExtents, DisplayMode, FontInfo, RenderBpp, RenderMode,
        VideoCard, VideoCardState, VideoCardStateEntry, VideoOption, VideoType,
    },
    devices::pic::Pic,
};
use marty_common::MartyHashMap;
use std::io::BufWriter;
use std::path::{Path, PathBuf};

// ---------------------------------------------------------------------------
// GTS-220 video trace (diagnostics for firmware RE).
//
// Enabled via environment variables, read once at card construction:
//   GTS220_VTRACE=1       log all video port I/O (decoded) + per-frame VRAM
//                         summary to 'gts220_vtrace.log' (working directory)
//   GTS220_VTRACE=<path>  same, but to a custom log path
//   GTS220_VTRACE_VRAM=1  additionally log EVERY VRAM read/write (large!)
//
// Line format:
//   f<frame> <event> — frame ticks every ~16.7ms of emulated time.
//   Port writes are decoded with standard VGA register names, e.g.:
//     f  123 P GC[5] (MODE) = 0x01        <- scroll latch-copy mode
//     f  123 P CRTC[CR0C] (SAH) = 0x00    <- start address high
//   Frame summaries:
//     f  123 F w=4821 wm0=99 wm1=4722 wm2=0 rows=0x0069-0x0F1B r=812 3da=441
//
// See also GTS220_STRACE (serial.rs) for the modem-side data log.
// ---------------------------------------------------------------------------
pub struct VTrace {
    writer: Option<BufWriter<std::fs::File>>,
    /// Log every VRAM read/write (very verbose).
    full_vram: bool,
    // Per-frame VRAM/port-read accumulators (summary mode).
    vram_writes: u32,
    wm_count: [u32; 8],
    row_min: u16,
    row_max: u16,
    vram_reads: u32,
    port_reads: [u32; 0x20], // 0x3C0-0x3DF
}

impl VTrace {
    fn from_env() -> Self {
        let (path, full_vram) = match std::env::var("GTS220_VTRACE") {
            Ok(v) if v == "1" || v.eq_ignore_ascii_case("true") => {
                (Some(PathBuf::from("gts220_vtrace.log")), false)
            }
            Ok(v) if !v.is_empty() => (Some(PathBuf::from(&v)), false),
            _ => (None, false),
        };
        let full_vram =
            full_vram || std::env::var("GTS220_VTRACE_VRAM").map(|v| v == "1").unwrap_or(false);
        let writer = path.and_then(|p| {
            match std::fs::File::create(&p) {
                Ok(f) => {
                    log::info!("GTS220: video trace enabled -> {}", p.display());
                    Some(BufWriter::with_capacity(1 << 16, f))
                }
                Err(e) => {
                    log::warn!("GTS220: video trace create failed ({}): {}", p.display(), e);
                    None
                }
            }
        });
        Self {
            writer,
            full_vram,
            vram_writes: 0,
            wm_count: [0; 8],
            row_min: u16::MAX,
            row_max: 0,
            vram_reads: 0,
            port_reads: [0; 0x20],
        }
    }

    #[inline]
    pub fn enabled(&self) -> bool {
        self.writer.is_some()
    }

    #[inline]
    pub fn full_vram(&self) -> bool {
        self.full_vram
    }

    pub fn line(&mut self, s: String) {
        if let Some(w) = self.writer.as_mut() {
            use std::io::Write;
            let _ = w.write_all(s.as_bytes());
            let _ = w.write_all(b"\n");
        }
    }

    pub fn bump_write(&mut self, wm: usize, row: u16) {
        self.vram_writes += 1;
        if wm < 8 {
            self.wm_count[wm] += 1;
        }
        self.row_min = self.row_min.min(row);
        self.row_max = self.row_max.max(row);
    }

    pub fn bump_read(&mut self) {
        self.vram_reads += 1;
    }

    pub fn bump_port_read(&mut self, port: u16) {
        if (0x3C0..0x3E0).contains(&port) {
            self.port_reads[(port - 0x3C0) as usize] += 1;
        }
    }

    /// Emit the per-frame summary and reset accumulators.
    pub fn frame_tick(&mut self, frame: u64) {
        if self.vram_writes > 0 || self.vram_reads > 0 {
            let reads = self
                .port_reads
                .iter()
                .enumerate()
                .filter(|(_, &n)| n > 0)
                .map(|(i, &n)| format!("{:03x}={}", 0x3C0 + i, n))
                .collect::<Vec<_>>()
                .join(" ");
            let line = format!(
                "f{:>6} F w={} wm0={} wm1={} wm2={} rows={:#06x}-{:#06x} vr={}{}",
                frame,
                self.vram_writes,
                self.wm_count[0],
                self.wm_count[1],
                self.wm_count[2],
                if self.vram_writes > 0 { self.row_min } else { 0 },
                if self.vram_writes > 0 { self.row_max } else { 0 },
                self.vram_reads,
                if reads.is_empty() { String::new() } else { format!(" pr[{reads}]") },
            );
            self.line(line);
        }
        self.vram_writes = 0;
        self.wm_count = [0; 8];
        self.row_min = u16::MAX;
        self.row_max = 0;
        self.vram_reads = 0;
        self.port_reads = [0; 0x20];
    }

    pub fn flush(&mut self) {
        if let Some(w) = self.writer.as_mut() {
            use std::io::Write;
            let _ = w.flush();
        }
    }
}

/// Standard VGA CRTC register names (index as programmed by the firmware).
fn crtc_name(idx: u8) -> &'static str {
    match idx {
        0x00 => "HT",       // Horizontal Total
        0x01 => "HDE",      // Horizontal Display End
        0x02 => "HBS",      // Horizontal Blank Start
        0x03 => "HBE",      // Horizontal Blank End
        0x04 => "HRS",      // Horizontal Retrace Start
        0x05 => "HRE",      // Horizontal Retrace End
        0x06 => "VT",       // Vertical Total
        0x07 => "OVR",      // Overflow
        0x08 => "PRS",      // Preset Row Scan
        0x09 => "MSL",      // Maximum Scan Line
        0x0A => "CS",       // Cursor Start
        0x0B => "CE",       // Cursor End
        0x0C => "SAH",      // Start Address High
        0x0D => "SAL",      // Start Address Low
        0x0E => "CLH",      // Cursor Location High
        0x0F => "CLL",      // Cursor Location Low
        0x10 => "VRS",      // Vertical Retrace Start
        0x11 => "VRE",      // Vertical Retrace End (+ WP)
        0x12 => "VDE",      // Vertical Display End
        0x13 => "OFS",      // Offset (logical line width)
        0x14 => "UL",       // Underline Location
        0x15 => "VBS",      // Start Vertical Blank
        0x16 => "VBE",      // End Vertical Blank
        0x17 => "MODE",     // CRTC Mode Control
        0x18 => "LC",       // Line Compare
        _ => "??",
    }
}

fn seq_name(idx: usize) -> &'static str {
    match idx {
        0 => "RST",  // Sequencer Reset
        1 => "CLK",  // Clocking Mode
        2 => "MM",   // Map Mask
        3 => "CHM",  // Character Map Select
        4 => "MEM",  // Memory Mode
        _ => "??",
    }
}

fn gc_name(idx: usize) -> &'static str {
    match idx {
        0 => "SR",   // Set/Reset
        1 => "ESR",  // Enable Set/Reset
        2 => "CC",   // Color Compare
        3 => "ROT",  // Data Rotate
        4 => "RM",   // Read Map Select
        5 => "MODE", // Graphics Mode
        6 => "MISC", // Miscellaneous
        7 => "CDC",  // Color Don't Care
        8 => "BM",   // Bit Mask
        _ => "??",
    }
}

fn atc_name(idx: u8) -> &'static str {
    match idx {
        0x00..=0x0F => "PAL",
        0x10 => "MODE", // Attribute Mode Control
        0x11 => "OSC",  // Overscan Color
        0x12 => "CPE",  // Color Plane Enable
        0x13 => "hPPS", // Horizontal PEL Panning
        0x14 => "CSEL", // Color Select
        _ => "??",
    }
}

pub const GTS_VRAM_BASE: usize = 0xA0000;
pub const GTS_VRAM_SIZE: usize = 0x10000; // per-plane aperture
pub const GTS_FS_BASE: usize = 0xB0000; // NVSRAM window (paged FS storage)
pub const GTS_STRIDE: usize = 80; // bytes per row (640 px, MSB first)
pub const GTS_W: u32 = 640;
pub const GTS_H: u32 = 520;
pub const GTS_FRAME_BYTES: usize = GTS_STRIDE * GTS_H as usize; // 44,000

pub const GTS_FRAME_US: f64 = 16_700.0; // ~60Hz assumed refresh (unverified)

pub struct Gts220Card {
    // NVSRAM window state (bus 0xB0000-0xBFFFF).
    // Storage = KM681000B 128KB battery-backed SRAM = 2 x 64KB banks.
    // Page 0 = canonical bank (chain @0xFE00, dir @0xF200),
    // page 1 = mirror bank (chain @FE00/F000, dir @E400/F200).
    // The FULL 64KB window is banked (incl. E000-FFFF): the dir mirror at
    // page1@F200 shares its offset with the page0 canonical, proving the
    // upper 16KB is banked too. The old "split" model (fixed E000-FFFF
    // RAM) is retired.
    nv: Vec<u8>,        // 128KB NVSRAM image (bank 0 + bank 1)
    fs_paged: Vec<u8>,  // 0x10000: current bank window copy
    fs_page: u8,        // current page (0x3E2 value & 7)
    fs_switches: usize, // total page-switch count (diagnostics)
    /// Battery-backup persistence: writes are flushed here periodically so a
    /// page switch (or power cycle) preserves SETUP$$$.DAT/HITEL*.TEL edits.
    nvs_save_path: Option<PathBuf>,
    nv_dirty: bool,
    nvs_save_accum_us: f64,

    vram: Vec<Vec<u8>>, // 4 planes
    latch: [u8; 4],

    // Registers (VGA-compatible subset, per Python model)
    seq: [u8; 8],
    seq_idx: usize,
    gc: [u8; 16],
    gc_idx: usize,
    read_map: usize,
    attr_ff: bool,
    attr_idx: u8,
    /// Attribute Controller registers (RE 2026-10-19: the firmware programs
    /// 21 ATC registers from the mode table — palette 00-0F identity, mode,
    /// overscan, color plane enable, hPMS, color select).
    atc_pal: [u8; 16],
    atc_mode: u8,
    atc_overscan: u8,
    atc_color_plane_en: u8,
    atc_hpp: u8,
    atc_color_select: u8,
    crtc_idx: u8,
    /// CRTC start address (registers 0x0C/0x0D) — hardware scroll base.
    /// Written by the firmware's scroll path; render() offsets rows by it.
    crtc_start_hi: u8,
    crtc_start_lo: u8,
    misc: u8,
    dac_idx: u8,
    dac_wr: u8,
    dac_sub: u8,
    /// DAC palette RAM: 256 entries × 3 channels, 6-bit per channel.
    /// RE 2026-10-19: the firmware loads a real palette at boot
    /// (standard-VGA-like values, NOT the EGA palette the Python model
    /// assumed) and re-programs it later — render() must follow the DAC.
    dac: Vec<u8>,
    vsync: u8,

    // Rendering state
    display_buf: Vec<u8>, // RGBA, 640x550
    dirty: bool,
    us_accum: f64,
    frame_count: u64,

    /// Whether FS window (0xB0000) writes are accepted (see new()).
    fs_writable: bool,

    extents: DisplayExtents,

    /// Diagnostic video trace (env-gated: GTS220_VTRACE / GTS220_VTRACE_VRAM).
    vtrace: VTrace,
}

impl Gts220Card {
    /// Write the live 128KB NVSRAM banks to the battery-backup sidecar file
    /// (`<image>.nvs`) if they changed. Atomic via tmp+rename.
    fn flush_nvs(&mut self) {
        if !self.nv_dirty {
            return;
        }
        if let Some(save_path) = &self.nvs_save_path {
            let tmp_path = save_path.with_extension("nvs.tmp");
            match std::fs::write(&tmp_path, &self.nv)
                .and_then(|_| std::fs::rename(&tmp_path, save_path))
            {
                Ok(_) => self.nv_dirty = false,
                Err(e) => {
                    // Not fatal — battery state persists in memory; retry next period.
                    log::warn!("GTS220: NVSRAM flush failed ({}): {}", save_path.display(), e);
                    let _ = std::fs::remove_file(&tmp_path);
                }
            }
        }
    }

    /// `fs_writable`: whether FS window writes are accepted. The validated
    /// boot path (STATE-0, 2026-10-14/15) runs with the SYNTHESIZED NVSRAM
    /// image mapped read-only: the firmware's SETUP$$$.DAT delete+recreate
    /// then corrupts the chain table it walks afterwards (the synthetic
    /// image was only ever validated against reads), stalling the boot in
    /// the mini-DOS chain walk (F088:0AD9). Real-machine battery SRAM dumps
    /// should enable writes once their layout is verified.
    pub fn new(rom_path: Option<&str>, fs_writable: bool) -> Self {
        // NVSRAM image: 128KB (KM681000B, battery-backed SRAM) = banks 0 + 1.
        // Page N reads/writes bank N (full 64KB window). The 512KB synthesized
        // image (datarom_v3.bin) supplies both banks; pages 2+ have no
        // physical bank and are never selected by the firmware (all 42
        // latch writes audited: 27x page 0, 15x page 1). A real-machine
        // SRAM dump will eventually replace this image.
        let image: Vec<u8> = match rom_path {
            Some(path) => std::fs::read(path)
                .unwrap_or_else(|e| panic!("GTS220 NVSRAM image read failed ('{path}'): {e}")),
            None => vec![0; 0x80000],
        };
        assert_eq!(image.len(), 0x80000, "GTS220 NVSRAM image must be 512KB");

        // Battery-backup persistence: an ".nvs" sidecar next to the image
        // holds the live SRAM state; if present it supersedes the pristine
        // image (a battery that kept its charge through the last session).
        let nvs_save_path = rom_path.map(|p| PathBuf::from(format!("{p}.nvs")));
        let mut nv = image[0..0x20000].to_vec(); // banks 0 + 1 only
        if let Some(save_path) = &nvs_save_path {
            if let Ok(saved) = std::fs::read(save_path) {
                if saved.len() == nv.len() {
                    log::info!(
                        "GTS220: restoring NVSRAM battery-backup image from {}",
                        save_path.display()
                    );
                    nv.copy_from_slice(&saved);
                }
                else {
                    log::warn!(
                        "GTS220: NVSRAM backup '{}' size mismatch ({} != {}), ignoring",
                        save_path.display(),
                        saved.len(),
                        nv.len()
                    );
                }
            }
        }
        let fs_paged = nv[0..0x10000].to_vec(); // bank 0 (canonical)

        let apertures = vec![DisplayAperture {
            w:     GTS_W,
            h:     GTS_H,
            x:     0,
            y:     0,
            debug: false,
        }];
        Self {
            nv,
            fs_paged,
            fs_page: 0,
            fs_switches: 0,
            nvs_save_path,
            nv_dirty: false,
            nvs_save_accum_us: 0.0,
            fs_writable,
            vram: vec![vec![0u8; GTS_VRAM_SIZE]; 4],
            latch: [0; 4],
            seq: [0; 8],
            seq_idx: 0,
            gc: [0; 16],
            gc_idx: 0,
            read_map: 0,
            attr_ff: false,
            attr_idx: 0,
            atc_pal: [0; 16],
            atc_mode: 0,
            atc_overscan: 0,
            atc_color_plane_en: 0,
            atc_hpp: 0,
            atc_color_select: 0,
            crtc_idx: 0,
            crtc_start_hi: 0,
            crtc_start_lo: 0,
            misc: 0,
            dac_idx: 0,
            dac_wr: 0,
            dac_sub: 0,
            dac: vec![0u8; 256 * 3],
            vsync: 0,
            display_buf: vec![0u8; (GTS_W * GTS_H * 4) as usize],
            dirty: true,
            us_accum: 0.0,
            frame_count: 0,
            extents: DisplayExtents {
                apertures,
                field_w: GTS_W,
                field_h: GTS_H,
                // NOTE: the direct-u32 render paths index the RGBA buffer as
                // u32 elements, so row_stride is in PIXELS (see
                // draw_vga_direct_u32_rgba), not bytes.
                row_stride: GTS_W as usize,
                double_scan: false,
                mode_byte: 0,
            },
            vtrace: VTrace::from_env(),
        }
    }

    /// Expand a 6-bit DAC value to 8-bit (bit replication, as the RAMDAC does).
    #[inline]
    fn expand6(v: u8) -> u8 {
        ((v as u16) << 2 | (v as u16) >> 4) as u8
    }

    /// Compose the 4 bitplanes into the RGBA display buffer.
    /// Bit i of pixel = plane i's bit, MSB-first within each byte.
    ///
    /// Color pipeline: 4-bit pixel → ATC color-plane enable → ATC palette
    /// → (color select adds DAC address bits 5-4) → DAC 6-bit RGB → 8-bit.
    pub fn render(&mut self) {
        // Build the 16-entry RGBA LUT from the live ATC/DAC state.
        let mut lut = [[0u8; 4]; 16];
        for (i, entry) in lut.iter_mut().enumerate() {
            let pix_masked = i as u8 & self.atc_color_plane_en & 0x0F;
            let atc = self.atc_pal[pix_masked as usize] as usize;
            let dac_idx = (atc & 0x3F) | (((self.atc_color_select & 0x03) as usize) << 4);
            let base = dac_idx * 3;
            *entry = [
                Self::expand6(self.dac[base]),
                Self::expand6(self.dac[base + 1]),
                Self::expand6(self.dac[base + 2]),
                255,
            ];
        }

        let mut o = 0usize;
        for y in 0..GTS_H as usize {
            let row = y * GTS_STRIDE;
            for byte_i in 0..GTS_STRIDE {
                let p0 = self.vram[0][row + byte_i];
                let p1 = self.vram[1][row + byte_i];
                let p2 = self.vram[2][row + byte_i];
                let p3 = self.vram[3][row + byte_i];
                for bit in (0..8).rev() {
                    let idx = ((p0 >> bit) & 1)
                        | (((p1 >> bit) & 1) << 1)
                        | (((p2 >> bit) & 1) << 2)
                        | (((p3 >> bit) & 1) << 3);
                    let c = &lut[idx as usize];
                    self.display_buf[o] = c[0];
                    self.display_buf[o + 1] = c[1];
                    self.display_buf[o + 2] = c[2];
                    self.display_buf[o + 3] = c[3];
                    o += 4;
                }
            }
        }
        self.frame_count += 1;
        self.dirty = false;
    }

    fn apply_write(&mut self, off: usize, data: u8) {
        let off = off % GTS_VRAM_SIZE;
        let wm = self.gc[5] & 7;
        let esr = self.gc[1];
        let sr = self.gc[0];
        let bm = self.gc[8];
        // NOTE: faithful to the Python model: map mask 0 -> all planes.
        let mm = if self.seq[2] != 0 { self.seq[2] } else { 0x0F };

        for p in 0..4usize {
            if mm & (1 << p) == 0 {
                continue;
            }
            let b: u8 = match wm {
                0 => {
                    if (esr >> p) & 1 != 0 {
                        if (sr >> p) & 1 != 0 {
                            0xFF
                        }
                        else {
                            0x00
                        }
                    }
                    else {
                        data
                    }
                }
                1 => self.latch[p],
                2 => {
                    if (data >> p) & 1 != 0 {
                        0xFF
                    }
                    else {
                        0x00
                    }
                }
                3 => {
                    let srb: u8 = if (sr >> p) & 1 != 0 { 0xFF } else { 0x00 };
                    (srb & data & bm) | (self.latch[p] & !bm)
                }
                _ => data,
            };
            let old = self.latch[p];
            self.vram[p][off] = (old & (0xFF ^ bm)) | (b & bm);
        }
        self.dirty = true;
    }

    fn port_write(&mut self, port: u16, v: u8) {
        // Video trace: decode every port write (index state is still the
        // pre-write value here, which is what data-port writes refer to).
        if self.vtrace.enabled() {
            let line = match port {
                0x3C0 => {
                    if self.attr_ff {
                        format!(
                            "f{:>6} P ATC data[{:02X} {}] = {:#04x}",
                            self.frame_count,
                            self.attr_idx,
                            atc_name(self.attr_idx),
                            v
                        )
                    }
                    else {
                        format!(
                            "f{:>6} P ATC index = {:#04x} ({})",
                            self.frame_count,
                            v & 0x1F,
                            atc_name(v & 0x1F)
                        )
                    }
                }
                0x3C2 => format!("f{:>6} P MISC = {:#04x}", self.frame_count, v),
                0x3C4 => format!(
                    "f{:>6} P SEQ index = {:#04x} ({})",
                    self.frame_count,
                    v & 7,
                    seq_name((v & 7) as usize)
                ),
                0x3C5 => format!(
                    "f{:>6} P SEQ[{:02X} {}] = {:#04x}",
                    self.frame_count,
                    self.seq_idx,
                    seq_name(self.seq_idx),
                    v
                ),
                0x3C6 => format!("f{:>6} P DAC pelmask = {:#04x}", self.frame_count, v),
                0x3C7 => format!("f{:>6} P DAC readidx = {:#04x}", self.frame_count, v),
                0x3C8 => format!("f{:>6} P DAC writeidx = {:#04x}", self.frame_count, v),
                0x3C9 => format!(
                    "f{:>6} P DAC[{:02x}.{}] = {:#04x}",
                    self.frame_count, self.dac_wr, self.dac_sub, v
                ),
                0x3CE => format!(
                    "f{:>6} P GC index = {:#04x} ({})",
                    self.frame_count,
                    v & 0xF,
                    gc_name((v & 0xF) as usize)
                ),
                0x3CF => format!(
                    "f{:>6} P GC[{:02X} {}] = {:#04x}",
                    self.frame_count,
                    self.gc_idx,
                    gc_name(self.gc_idx),
                    v
                ),
                0x3D4 => format!("f{:>6} P CRTC index = {:#04x}", self.frame_count, v),
                0x3D5 => format!(
                    "f{:>6} P CRTC[CR{:02X} {}] = {:#04x}",
                    self.frame_count,
                    self.crtc_idx,
                    crtc_name(self.crtc_idx),
                    v
                ),
                0x3DA => format!("f{:>6} P 3DA write = {:#04x}", self.frame_count, v),
                0x3E2 => format!("f{:>6} P FS page latch = {}", self.frame_count, v & 7),
                _ => format!("f{:>6} P {:03x} = {:#04x}", self.frame_count, port, v),
            };
            self.vtrace.line(line);
        }
        match port {
            0x3C0 => {
                if self.attr_ff {
                    // Attribute data write: store ATC registers (RE: the
                    // firmware programs 21 ATC regs from the mode table).
                    match self.attr_idx {
                        0..=15 => self.atc_pal[self.attr_idx as usize] = v,
                        0x10 => self.atc_mode = v,
                        0x11 => self.atc_overscan = v,
                        0x12 => self.atc_color_plane_en = v,
                        0x13 => self.atc_hpp = v,
                        0x14 => self.atc_color_select = v,
                        _ => {}
                    }
                    self.dirty = true;
                }
                else {
                    self.attr_idx = v & 0x1F;
                }
                self.attr_ff = !self.attr_ff;
            }
            0x3C2 => {
                self.misc = v;
            }
            0x3C4 => {
                self.seq_idx = (v & 7) as usize;
            }
            0x3C5 => {
                self.seq[self.seq_idx] = v;
            }
            0x3C7 => {
                self.dac_idx = v;
                self.dac_sub = 0;
            }
            0x3C8 => {
                self.dac_wr = v;
                self.dac_sub = 0;
            }
            0x3C9 => {
                // DAC palette data write: 6-bit per channel, auto-increment
                // every 3rd byte, index wraps mod 256.
                let base = (self.dac_wr as usize) * 3;
                self.dac[base + self.dac_sub as usize] = v & 0x3F;
                self.dac_sub += 1;
                if self.dac_sub == 3 {
                    self.dac_sub = 0;
                    self.dac_wr = self.dac_wr.wrapping_add(1);
                }
                self.dirty = true;
            }
            0x3CE => {
                self.gc_idx = (v & 0xF) as usize;
            }
            0x3CF => {
                self.gc[self.gc_idx] = v;
                if self.gc_idx == 4 {
                    self.read_map = (v & 3) as usize;
                }
            }
            0x3D4 => {
                self.crtc_idx = v;
            }
            0x3D5 => {
                // CRTC data write. Static screens never program the CRTC
                // (RE: fixed-mode), but the online/scroll path is expected
                // to use the start address registers (0x0C/0x0D) for
                // hardware scrolling — VRAM has 64KB/plane against a 44KB
                // screen (269 hidden rows). Track the start address.
                match self.crtc_idx {
                    0x0C => self.crtc_start_hi = v,
                    0x0D => self.crtc_start_lo = v,
                    _ => {}
                }
            }
            0x3E2 => {
                // NVSRAM page latch (KM681000B 128KB = banks 0 + 1; RE-verified
                // 2026-10-14/15). Page = v & 7, full 64KB window banked.
                // Firmware uses 0/1 (canonical/mirror); FS init also writes 2
                // (meaning unresolved — bank 2 does not physically exist).
                let page = v & 7;
                if page != self.fs_page {
                    self.fs_page = page;
                    self.fs_switches += 1;
                    if page <= 1 {
                        let base = (page as usize) * 0x10000;
                        self.fs_paged.copy_from_slice(&self.nv[base..base + 0x10000]);
                    }
                    // page >= 2: no physical bank; window unchanged (floating).
                }
            }
            _ => {}
        }
    }
}

impl Default for Gts220Card {
    fn default() -> Self {
        Self::new(None, false)
    }
}

impl Drop for Gts220Card {
    fn drop(&mut self) {
        // Last-chance battery flush on teardown (machine quit/rebuild).
        self.flush_nvs();
        self.vtrace.flush();
    }
}

// ---------------------------------------------------------------------------
// IoDevice: video ports 0x3C0-0x3DF
// ---------------------------------------------------------------------------
impl IoDevice for Gts220Card {
    fn read_u8(&mut self, port: u16, _delta: DeviceRunTimeUnit) -> u8 {
        // Video trace: count port reads (polled ports like 0x3DA dominate);
        // individual read lines only in full-VRAM mode.
        let traced = self.vtrace.enabled();
        if traced {
            self.vtrace.bump_port_read(port);
        }
        let ret = if port == 0x3DA {
            // Python model: vsync bit (3) toggles on each read, DE (bit 0) always set.
            // Real VGA: reading ISR1 also resets the ATC index/data flip-flop
            // to the "index" state — the firmware reads 0x3DA before ATC
            // programming for exactly this purpose.
            self.vsync ^= 8;
            self.attr_ff = false;
            (self.vsync | 0x01) & 0xFF
        }
        else if port == 0x3C1 {
            // ATC data read-back.
            let v = match self.attr_idx {
                0..=15 => self.atc_pal[self.attr_idx as usize],
                0x10 => self.atc_mode,
                0x11 => self.atc_overscan,
                0x12 => self.atc_color_plane_en,
                0x13 => self.atc_hpp,
                0x14 => self.atc_color_select,
                _ => 0,
            };
            v
        }
        else if port == 0x3C9 {
            // DAC palette data read (6-bit per channel, auto-increment).
            let base = (self.dac_idx as usize) * 3;
            let v = self.dac[base + self.dac_sub as usize];
            self.dac_sub += 1;
            if self.dac_sub == 3 {
                self.dac_sub = 0;
                self.dac_idx = self.dac_idx.wrapping_add(1);
            }
            v
        }
        else if (0x3B0..=0x3DF).contains(&port) {
            0x00
        }
        else if port == 0x3E2 {
            // NVSRAM page latch read: firmware never reads it; Python
            // returns 0xFF (unmapped-port behavior).
            0xFF
        }
        else {
            NO_IO_BYTE
        };
        if traced && self.vtrace.full_vram() {
            self.vtrace
                .line(format!("f{:>6} R {:03x} -> {:#04x}", self.frame_count, port, ret));
        }
        ret
    }

    fn write_u8(
        &mut self,
        port: u16,
        data: u8,
        _bus: Option<&mut BusInterface>,
        _delta: DeviceRunTimeUnit,
        _analyzer: Option<&mut LogicAnalyzer>,
    ) {
        if (0x3C0..=0x3DF).contains(&port) {
            self.port_write(port, data);
        }
        else if port == 0x3E2 {
            self.port_write(port, data);
        }
    }

    fn port_list(&self) -> Vec<(String, u16)> {
        vec![
            ("GTS220 Attr Controller/Index".to_string(), 0x3C0),
            ("GTS220 Attr Controller/Data".to_string(), 0x3C1),
            ("GTS220 Misc Output".to_string(), 0x3C2),
            ("GTS220 Sequencer Index".to_string(), 0x3C4),
            ("GTS220 Sequencer Data".to_string(), 0x3C5),
            ("GTS220 DAC PEL Mask".to_string(), 0x3C6),
            ("GTS220 DAC Read Index".to_string(), 0x3C7),
            ("GTS220 DAC Write Index".to_string(), 0x3C8),
            ("GTS220 DAC Data".to_string(), 0x3C9),
            ("GTS220 GC Index".to_string(), 0x3CE),
            ("GTS220 GC Data".to_string(), 0x3CF),
            ("GTS220 CRTC Index".to_string(), 0x3D4),
            ("GTS220 CRTC Data".to_string(), 0x3D5),
            ("GTS220 Input Status 1".to_string(), 0x3DA),
            ("GTS220 NVSRAM Page Latch".to_string(), 0x3E2),
        ]
    }
}

// ---------------------------------------------------------------------------
// MemoryMappedDevice: planar VRAM @ 0xA0000 + data-ROM/FS window @ 0xB0000
// ---------------------------------------------------------------------------
impl MemoryMappedDevice for Gts220Card {
    fn get_read_wait(&mut self, _address: usize, _cycles: u32) -> u32 {
        0
    }

    fn mmio_read_u8(&mut self, address: usize, _cycles: u32, _cpumem: Option<&[u8]>) -> (u8, u32) {
        if (GTS_VRAM_BASE..GTS_VRAM_BASE + GTS_VRAM_SIZE).contains(&address) {
            let off = (address - GTS_VRAM_BASE) % GTS_VRAM_SIZE;
            if self.vtrace.enabled() {
                if self.vtrace.full_vram() {
                    self.vtrace.line(format!(
                        "f{:>6} Rv {:04x} (rm{})",
                        self.frame_count, off, self.read_map
                    ));
                }
                self.vtrace.bump_read();
            }
            // Python hook_vram_read: every read loads all four latches.
            for p in 0..4 {
                self.latch[p] = self.vram[p][off];
            }
            (self.vram[self.read_map][off], 0)
        }
        else if (GTS_FS_BASE..GTS_FS_BASE + 0x10000).contains(&address) {
            let off = address - GTS_FS_BASE;
            (self.fs_paged[off], 0)
        }
        else {
            (0xFF, 0)
        }
    }

    fn mmio_read_u16(&mut self, address: usize, cycles: u32, cpumem: Option<&[u8]>) -> (u16, u32) {
        let (lo, w1) = MemoryMappedDevice::mmio_read_u8(self, address, cycles, cpumem);
        let (hi, w2) = MemoryMappedDevice::mmio_read_u8(self, address + 1, cycles, cpumem);
        ((hi as u16) << 8 | lo as u16, w1 + w2)
    }

    fn mmio_peek_u8(&self, address: usize, _cpumem: Option<&[u8]>) -> u8 {
        if (GTS_VRAM_BASE..GTS_VRAM_BASE + GTS_VRAM_SIZE).contains(&address) {
            let off = (address - GTS_VRAM_BASE) % GTS_VRAM_SIZE;
            self.vram[self.read_map][off]
        }
        else if (GTS_FS_BASE..GTS_FS_BASE + 0x10000).contains(&address) {
            let off = address - GTS_FS_BASE;
            self.fs_paged[off]
        }
        else {
            0xFF
        }
    }

    fn mmio_peek_u16(&self, address: usize, _cpumem: Option<&[u8]>) -> u16 {
        let lo = MemoryMappedDevice::mmio_peek_u8(self, address, _cpumem);
        let hi = MemoryMappedDevice::mmio_peek_u8(self, address + 1, _cpumem);
        (hi as u16) << 8 | lo as u16
    }

    fn get_write_wait(&mut self, _address: usize, _cycles: u32) -> u32 {
        0
    }

    fn mmio_write_u8(&mut self, address: usize, byte: u8, _cycles: u32, _cpumem: Option<&mut [u8]>) -> u32 {
        if (GTS_VRAM_BASE..GTS_VRAM_BASE + GTS_VRAM_SIZE).contains(&address) {
            let off = (address - GTS_VRAM_BASE) % GTS_VRAM_SIZE;
            if self.vtrace.enabled() {
                let row = (off / GTS_STRIDE) as u16;
                if self.vtrace.full_vram() {
                    self.vtrace.line(format!(
                        "f{:>6} W {:04x} = {:02x} wm{} bm{:02x} sr{:02x} mm{:02x}",
                        self.frame_count, off, byte, self.gc[5] & 7, self.gc[8], self.gc[0], self.seq[2]
                    ));
                }
                self.vtrace.bump_write((self.gc[5] & 7) as usize, row);
            }
            self.apply_write(off, byte);
        }
        else if (GTS_FS_BASE..GTS_FS_BASE + 0x10000).contains(&address) {
            if !self.fs_writable {
                // Battery SRAM image mapped read-only: writes dropped (the
                // validated boot path depends on this with the synthesized
                // image — see new()).
                return 0;
            }
            let off = address - GTS_FS_BASE;
            self.fs_paged[off] = byte;
            // Write-through to the selected bank. Without this, a page
            // switch reloads the window from `nv` and silently discards
            // every write made in the previous page (the FS keeps the
            // canonical bank on page 0 but updates its mirrors on page 1,
            // so cross-page persistence is mandatory for correct 3-way
            // checksum voting and SETUP$$$.DAT edits).
            if self.fs_page <= 1 {
                let base = (self.fs_page as usize) * 0x10000;
                self.nv[base + off] = byte;
            }
            // page >= 2: no physical bank selected; the write is dropped,
            // matching the floating window behavior of the latch model.
            self.nv_dirty = true;
        }
        0
    }

    fn mmio_write_u16(&mut self, address: usize, data: u16, _cycles: u32, _cpumem: Option<&mut [u8]>) -> u32 {
        // Python: two sequential byte writes, second offset wraps within the region.
        MemoryMappedDevice::mmio_write_u8(self, address, (data & 0xFF) as u8, 0, None);
        MemoryMappedDevice::mmio_write_u8(self, address + 1, ((data >> 8) & 0xFF) as u8, 0, None);
        0
    }

    fn get_mapping(&self) -> Vec<MemRangeDescriptor> {
        vec![
            MemRangeDescriptor {
                address: GTS_VRAM_BASE,
                size: GTS_VRAM_SIZE,
                cycle_cost: 0,
                read_only: false,
                priority: 1,
            },
            MemRangeDescriptor {
                address: GTS_FS_BASE,
                size: 0x10000,
                cycle_cost: 0,
                read_only: !self.fs_writable,
                priority: 1,
            },
        ]
    }
}

// ---------------------------------------------------------------------------
// VideoCard trait
// ---------------------------------------------------------------------------
impl VideoCard for Gts220Card {
    fn set_video_option(&mut self, _opt: VideoOption) {}

    fn video_type(&self) -> VideoType {
        VideoType::Gts220
    }

    fn render_mode(&self) -> RenderMode {
        RenderMode::Direct
    }

    fn render_depth(&self) -> RenderBpp {
        RenderBpp::ThirtyTwo
    }

    fn display_mode(&self) -> DisplayMode {
        DisplayMode::Mode6HiResGraphics
    }

    fn set_clocking_mode(&mut self, _mode: ClockingMode) {}

    fn display_size(&self) -> (u32, u32) {
        (GTS_W, GTS_H)
    }

    fn display_extents(&self) -> &DisplayExtents {
        &self.extents
    }

    fn list_display_apertures(&self) -> Vec<DisplayApertureDesc> {
        vec![DisplayApertureDesc {
            name:      "GTS220 Full",
            aper_enum: DisplayApertureType::Full,
        }]
    }

    fn display_apertures(&self) -> Vec<DisplayAperture> {
        self.extents.apertures.clone()
    }

    fn overscan_color(&self) -> u8 {
        0
    }

    fn buf(&self, _buf_select: BufferSelect) -> &[u8] {
        &self.display_buf
    }

    fn display_buf(&self) -> &[u8] {
        &self.display_buf
    }

    fn clock_divisor(&self) -> u32 {
        1
    }

    fn sync(&self) -> (bool, bool, bool, bool) {
        (true, true, true, false)
    }

    fn beam_pos(&self) -> Option<(u32, u32)> {
        None
    }

    fn scanline(&self) -> u32 {
        0
    }

    fn is_scanline_doubled(&self) -> bool {
        false
    }

    fn refresh_rate(&self) -> f32 {
        60.0
    }

    fn start_address(&self) -> u16 {
        0
    }

    fn is_40_columns(&self) -> bool {
        false
    }

    fn is_in_graphics_mode(&self) -> bool {
        true
    }

    fn cursor_info(&self) -> CursorInfo {
        CursorInfo {
            addr:       0,
            pos_x:      0,
            pos_y:      0,
            line_start: 0,
            line_end:   0,
            visible:    false,
        }
    }

    fn current_font(&self) -> Option<FontInfo> {
        None
    }

    fn character_height(&self) -> u8 {
        16
    }

    fn palette(&self) -> Option<Vec<[u8; 4]>> {
        // Live DAC-derived palette (16 entries via ATC identity path).
        let mut out = Vec::with_capacity(16);
        for i in 0..16u8 {
            let pix = i & self.atc_color_plane_en & 0x0F;
            let atc = self.atc_pal[pix as usize] as usize;
            let dac_idx = (atc & 0x3F) | (((self.atc_color_select & 0x03) as usize) << 4);
            let base = dac_idx * 3;
            out.push([
                Self::expand6(self.dac[base]),
                Self::expand6(self.dac[base + 1]),
                Self::expand6(self.dac[base + 2]),
                255,
            ]);
        }
        Some(out)
    }

    fn videocard_string_state(&self) -> VideoCardState {
        let mut state = MartyHashMap::default();
        state.insert(
            "GTS220".to_string(),
            vec![
                ("Map Mask".to_string(), VideoCardStateEntry::Value8(self.seq[2])),
                (
                    "Read Map".to_string(),
                    VideoCardStateEntry::Value8(self.read_map as u8),
                ),
                ("GC Mode".to_string(), VideoCardStateEntry::Value8(self.gc[5])),
                ("Bit Mask".to_string(), VideoCardStateEntry::Value8(self.gc[8])),
                (
                    "FS Page".to_string(),
                    VideoCardStateEntry::Value8(self.fs_page),
                ),
                (
                    "FS Switches".to_string(),
                    VideoCardStateEntry::Value8(self.fs_switches.min(255) as u8),
                ),
                (
                    "CRTC Start".to_string(),
                    VideoCardStateEntry::Value8(
                        ((((self.crtc_start_hi as usize) << 8) | self.crtc_start_lo as usize)
                            .min(255 * 256)
                            / 256) as u8,
                    ),
                ),
            ],
        );
        state
    }

    fn run(&mut self, time: DeviceRunTimeUnit, _pic: &mut Option<Box<Pic>>, _cpumem: Option<&[u8]>) {
        if let DeviceRunTimeUnit::Microseconds(us) = time {
            self.us_accum += us;
            if self.us_accum >= GTS_FRAME_US {
                self.us_accum = 0.0;
                if self.dirty {
                    self.render();
                }
                else {
                    self.frame_count += 1;
                }
                // Video trace: per-frame VRAM/port summary + periodic flush.
                if self.vtrace.enabled() {
                    self.vtrace.frame_tick(self.frame_count);
                    if self.frame_count % 16 == 0 {
                        self.vtrace.flush();
                    }
                }
                // Periodic battery-backup flush (~1s cadence).
                self.nvs_save_accum_us += GTS_FRAME_US;
                if self.nvs_save_accum_us >= 1_000_000.0 {
                    self.nvs_save_accum_us = 0.0;
                    self.flush_nvs();
                }
            }
        }
    }

    fn debug_tick(&mut self, _ticks: u32, _cpumem: Option<&[u8]>) {}

    fn reset(&mut self) {
        self.seq = [0; 8];
        self.gc = [0; 16];
        self.read_map = 0;
        self.attr_ff = false;
        self.attr_idx = 0;
        self.atc_pal = [0; 16];
        self.atc_mode = 0;
        self.atc_overscan = 0;
        self.atc_color_plane_en = 0;
        self.atc_hpp = 0;
        self.atc_color_select = 0;
        self.crtc_idx = 0;
        self.crtc_start_hi = 0;
        self.crtc_start_lo = 0;
        self.misc = 0;
        self.dac_idx = 0;
        self.dac_wr = 0;
        self.dac_sub = 0;
        self.dac.iter_mut().for_each(|b| *b = 0);
        self.vsync = 0;
        self.dirty = true;
    }

    fn pixel_raw(&self, x: u32, y: u32) -> u8 {
        if x >= GTS_W || y >= GTS_H {
            return 0;
        }
        let off = (y as usize) * GTS_STRIDE + (x as usize / 8);
        let bit = 7 - (x as usize % 8);
        ((self.vram[0][off] >> bit) & 1)
            | (((self.vram[1][off] >> bit) & 1) << 1)
            | (((self.vram[2][off] >> bit) & 1) << 2)
            | (((self.vram[3][off] >> bit) & 1) << 3)
    }

    fn pixel(&self, x: u32, y: u32) -> &[u8] {
        let o = ((y * GTS_W + x) * 4) as usize;
        &self.display_buf[o..o + 4]
    }

    fn plane_slice(&self, plane: usize) -> &[u8] {
        &self.vram[plane % 4]
    }

    fn frame_count(&self) -> u64 {
        self.frame_count
    }

    fn dump_mem(&self, path: &Path) {
        for (i, plane) in self.vram.iter().enumerate() {
            let p = path.with_extension(format!("plane{}.bin", i));
            let _ = std::fs::write(p, plane);
        }
    }

    fn write_trace_log(&mut self, _msg: String) {}

    fn trace_flush(&mut self) {}

    fn get_text_mode_strings(&self) -> Vec<String> {
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn poke(card: &mut Gts220Card, port: u16, v: u8) {
        card.write_u8(port, v, None, DeviceRunTimeUnit::Microseconds(0.0), None);
    }

    /// A write into the 0xB0000 window must survive a page switch and return:
    /// the FS keeps the canonical bank on page 0 but updates mirrors on
    /// page 1, so writes must land in the backing `nv` banks.
    #[test]
    fn fs_write_survives_page_switch() {
        let mut card = Gts220Card::new(None, true); // writable: mechanics test

        // Write on page 0.
        poke(&mut card, 0x3E2, 0);
        card.mmio_write_u8(GTS_FS_BASE + 0x1234, 0xAB, 0, None);

        // Switch to page 1 and write the same offset.
        poke(&mut card, 0x3E2, 1);
        card.mmio_write_u8(GTS_FS_BASE + 0x1234, 0xCD, 0, None);

        // Back to page 0: original byte must be intact.
        poke(&mut card, 0x3E2, 0);
        let (v, _) = card.mmio_read_u8(GTS_FS_BASE + 0x1234, 0, None);
        assert_eq!(v, 0xAB, "page-0 write lost across page switch");

        // Page 1 write must also persist.
        poke(&mut card, 0x3E2, 1);
        let (v, _) = card.mmio_read_u8(GTS_FS_BASE + 0x1234, 0, None);
        assert_eq!(v, 0xCD, "page-1 write lost across page switch");

        // Page >= 2 has no physical bank: writes dropped, window floats.
        poke(&mut card, 0x3E2, 2);
        card.mmio_write_u8(GTS_FS_BASE + 0x1234, 0xEE, 0, None);
        poke(&mut card, 0x3E2, 1);
        let (v, _) = card.mmio_read_u8(GTS_FS_BASE + 0x1234, 0, None);
        assert_eq!(v, 0xCD, "page-2 write must not corrupt any bank");
    }

    /// Battery-backup persistence: writes flush to a `<image>.nvs` sidecar
    /// and supersede the pristine image on the next construction.
    #[test]
    fn nvs_sidecar_roundtrip() {
        let dir = std::env::temp_dir().join("gts220_nvs_roundtrip");
        let _ = std::fs::create_dir_all(&dir);
        let img = dir.join("nvsram.bin");
        std::fs::write(&img, vec![0u8; 0x80000]).unwrap();
        let sidecar = dir.join("nvsram.bin.nvs");
        let _ = std::fs::remove_file(&sidecar);

        {
            let mut card = Gts220Card::new(Some(img.to_str().unwrap()), true);
            card.mmio_write_u8(GTS_FS_BASE, 0x5A, 0, None);
            card.flush_nvs();
        } // Drop also flushes.

        let saved = std::fs::read(&sidecar).expect("sidecar must exist after flush");
        assert_eq!(saved.len(), 0x20000);
        assert_eq!(saved[0], 0x5A, "byte must reach the sidecar");

        // Fresh construction restores battery state over the image.
        let mut card = Gts220Card::new(Some(img.to_str().unwrap()), true);
        let (v, _) = card.mmio_read_u8(GTS_FS_BASE, 0, None);
        assert_eq!(v, 0x5A, "battery-backup state must survive a power cycle");

        let _ = std::fs::remove_dir_all(&dir);
    }
}
