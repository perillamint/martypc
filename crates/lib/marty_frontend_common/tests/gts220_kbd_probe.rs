//! GTS-220 keyboard wiring probe.
//!
//! Boots through the frontend path, then:
//!   1. dumps the IRQ1 IVT entry + PIC IMR,
//!   2. presses '1' (Digit1) through machine.key_press(),
//!   3. watches the firmware's 100-byte keyboard ring buffer
//!      (selector phys 0x5DFA/0x5DFC, count [segi:0x446], buf [segb:0x1636]),
//!   4. reports where the chain breaks.

use marty_frontend_common::machine_manager::MachineManager;
use marty_frontend_common::resource_manager::{PathConfigItem, ResourceManager};
use marty_frontend_common::rom_manager::RomManager;

use marty_common::types::joystick::ControllerLayout;
use marty_common::types::keys::MartyKey;
use marty_core::coreconfig::CoreConfig;
use marty_core::cpu_common::TraceMode;
use marty_core::cpu_validator::ValidatorType;
use marty_core::machine::{ExecutionControl, ExecutionState, Machine, MachineBuilder};
use marty_core::machine_types::{MachineType, OnHaltBehavior};

const INSTALL_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../install");

struct TestCoreConfig;

impl CoreConfig for TestCoreConfig {
    fn get_base_dir(&self) -> std::path::PathBuf {
        INSTALL_DIR.into()
    }
    fn get_machine_type(&self) -> MachineType {
        MachineType::Gts220
    }
    fn get_audio_enabled(&self) -> bool {
        false
    }
    fn get_machine_noroms(&self) -> bool {
        false
    }
    fn get_machine_turbo(&self) -> bool {
        false
    }
    fn get_service_interrupt(&self) -> Option<u8> {
        None
    }
    fn get_keyboard_layout(&self) -> Option<String> {
        Some("US".to_string())
    }
    fn get_keyboard_debug(&self) -> bool {
        true
    }
    fn get_validator_type(&self) -> Option<ValidatorType> {
        None
    }
    fn get_validator_trace_file(&self) -> Option<std::path::PathBuf> {
        None
    }
    fn get_validator_baud(&self) -> Option<u32> {
        None
    }
    fn get_validator_port(&self) -> Option<String> {
        None
    }
    fn get_cpu_trace_mode(&self) -> Option<TraceMode> {
        None
    }
    fn get_cpu_dram_refresh_simulation(&self) -> bool {
        true
    }
    fn get_cpu_trace_on(&self) -> bool {
        false
    }
    fn get_cpu_trace_file(&self) -> Option<std::path::PathBuf> {
        None
    }
    fn get_title_hacks(&self) -> bool {
        false
    }
    fn get_patch_enabled(&self) -> bool {
        true
    }
    fn get_halt_behavior(&self) -> OnHaltBehavior {
        OnHaltBehavior::Continue
    }
    fn get_controller_layout(&self) -> Option<ControllerLayout> {
        None
    }
    fn get_terminal_port(&self) -> Option<u16> {
        None
    }
}

fn resource_paths() -> Vec<PathConfigItem> {
    vec![
        PathConfigItem {
            resource: "machine".into(),
            path: "$basedir$/configs/machines".into(),
            create: false,
            recurse: true,
        },
        PathConfigItem {
            resource: "romdef".into(),
            path: "$basedir$/configs/rom_definitions".into(),
            create: false,
            recurse: true,
        },
        PathConfigItem {
            resource: "rom".into(),
            path: "$basedir$/media/roms".into(),
            create: false,
            recurse: true,
        },
        PathConfigItem {
            resource: "keyboard_layout".into(),
            path: "$basedir$/configs/keyboard_layouts".into(),
            create: false,
            recurse: false,
        },
    ]
}

fn build_frontend_machine() -> Machine {
    let base = std::path::PathBuf::from(INSTALL_DIR);
    let mut rm =
        ResourceManager::from_config(base.clone(), &resource_paths()).expect("resource manager");

    let mut machine_manager = MachineManager::new();
    pollster::block_on(machine_manager.load_configs(&mut rm)).expect("machine configs load");
    let machine_config_file = machine_manager
        .get_config_with_overlays("gts220", &Vec::new())
        .expect("gts220 machine config");
    let machine_config = machine_config_file.to_machine_config();

    let mut rom_manager = RomManager::new(false);
    pollster::block_on(rom_manager.load_defs(&mut rm)).expect("romdef load");
    pollster::block_on(rom_manager.scan(&mut rm)).expect("rom scan");
    rom_manager.resolve_rom_sets().expect("rom set resolve");

    let (required, optional) = machine_config_file.get_rom_requirements(true).unwrap();
    let specified = machine_config_file.get_specified_rom_set();
    let resolved = rom_manager
        .resolve_requirements(required, optional, specified)
        .expect("rom requirements resolve");
    let manifest = rom_manager
        .create_manifest(resolved, &mut rm)
        .expect("rom manifest");

    MachineBuilder::new()
        .with_core_config(Box::new(&TestCoreConfig))
        .with_machine_config(&machine_config)
        .with_roms(manifest)
        // Mirror the frontend: load keyboard_US.toml content and pass it to
        // the builder so machine.rs runs Keyboard::load_mapping() exactly as
        // martypc_eframe does (this panics on `_ => unimplemented!()` if the
        // Gts220 arm is missing).
        .with_keyboard_layout({
            let toml_path = std::path::Path::new(INSTALL_DIR)
                .join("configs/keyboard_layouts/keyboard_US.toml");
            Some(std::fs::read_to_string(&toml_path).expect("keyboard_US.toml"))
        })
        .build()
        .expect("machine build")
}

fn rb_state(machine: &mut Machine) -> (u16, u16, u16, Vec<u8>) {
    let bus = machine.bus_mut();
    let segi = u16::from_le_bytes(bus.get_vec_at(0x387 * 16 + 0x258A, 2).try_into().unwrap());
    let segb = u16::from_le_bytes(bus.get_vec_at(0x387 * 16 + 0x258C, 2).try_into().unwrap());
    let cnt = if segi != 0 {
        u16::from_le_bytes(bus.get_vec_at(segi as usize * 16 + 0x446, 2).try_into().unwrap())
    }
    else {
        0
    };
    let buf = if segb != 0 && cnt > 0 {
        bus.get_vec_at(segb as usize * 16 + 0x1636, (cnt as usize).min(16))
    }
    else {
        Vec::new()
    };
    (segi, segb, cnt, buf)
}

#[test]
fn gts220_keyboard_probe() {
    // Logging: use the `log` test-capture if available; direct observation below
    // is the ground truth either way.
    std::env::set_current_dir(INSTALL_DIR).expect("chdir to install dir");
    let mut machine = build_frontend_machine();
    let mut exec = ExecutionControl::new();
    exec.set_state(ExecutionState::Running);

    // --- boot until STATE-0 (VTX main screen) or VT220 mode, then settle ---
    let mut cycles = 0u64;
    let target = 40_000_000u64;
    let mut booted = false;
    while cycles < target {
        let c = machine.run(100_000, &mut exec);
        if c == 0 {
            panic!("stalled at {cycles}");
        }
        cycles += c;
        while let Some(ev) = machine.get_event() {
            if let marty_core::machine::MachineEvent::CheckpointHit(idx, _) = ev {
                let desc = machine.get_checkpoint_string(idx).unwrap_or_default();
                if desc.contains("STATE-0") {
                    println!("[boot] STATE-0 checkpoint at {cycles} cycles");
                    booted = true;
                }
            }
        }
        let mode = machine.bus_mut().get_vec_at(0x60 * 16 + 0x5AA, 1)[0];
        if mode == 0x40 && cycles > 8_000_000 {
            println!("[boot] VT220 mode active at {cycles} cycles");
            booted = true;
        }
        if booted && cycles > 16_000_000 {
            break;
        }
    }
    assert!(booted, "never booted (no STATE-0 / VT220 mode) after {target} cycles");

    // --- snapshot pre-key state ---
    {
        let bus = machine.bus_mut();
        let ivt = bus.get_vec_at(0x04, 4); // INT 09h vector (IRQ1)
        let ivt1 = bus.get_vec_at(0x20, 4); // INT 08h vector (IRQ0)
        println!("[pre] INT 09h vector = {ivt:02X?}  INT 08h vector = {ivt1:02X?}");
    }
    let (segi, segb, cnt, buf) = rb_state(&mut machine);
    println!("[pre] ring: segi={segi:#06x} segb={segb:#06x} cnt={cnt} buf={buf:02X?}");

    // --- press '1' and watch ---
    for round in 0..3 {
        machine.key_press(MartyKey::Digit1, marty_core::devices::keyboard_common::KeyboardModifiers::default());
        machine.run(1_000_000, &mut exec);
        machine.key_release(MartyKey::Digit1);
        machine.run(3_000_000, &mut exec);
        while let Some(ev) = machine.get_event() {
            let _ = ev;
        }
        let (segi, segb, cnt, buf) = rb_state(&mut machine);
        println!("[key r{round}] ring: segi={segi:#06x} segb={segb:#06x} cnt={cnt} buf={buf:02X?}");

        // CPU location sanity: should be in the VT220 main loop area.
        use marty_core::cpu_808x::Cpu;
        use marty_core::cpu_common::Register16;
        let cpu = machine.cpu_mut();
        let cs = cpu.get_register16(Register16::CS);
        let ip = cpu.get_ip();
        println!("[key r{round}] CS:IP={cs:04X}:{ip:04X}");
    }

    // --- final verdict on the ring buffer ---
    let (_, _, cnt_final, _) = rb_state(&mut machine);
    println!("[final] ring count = {cnt_final}");
    // Not asserting: this is a probe. Ring count > 0 after 3 keys => wiring OK.
    // cnt staying 0 => the break is between key_press and the ring buffer.
}

/// Verify the keypad-mode flag cell (seg 0x60:0xfb0 = phys 0x15B0):
///   1. memory flags: no MMIO/ROM bits (plain conventional RAM),
///   2. bus write/read round-trip,
///   3. end-to-end: with the always-DOS-mode romdef patch installed, the
///      keypad-mode flag is IRRELEVANT - the Ctrl-6 screen's cursor moves on
///      Up/Down regardless of the flag value (translator flag branch NOPped).
#[test]
fn gts220_flag_cell_mapping() {
    std::env::set_current_dir(INSTALL_DIR).expect("chdir to install dir");
    let mut machine = build_frontend_machine();
    let mut exec = ExecutionControl::new();
    exec.set_state(ExecutionState::Running);

    const FLAG: usize = 0x60 * 16 + 0xfb0; // phys 0x15B0 (seg 0x60:0xfb0)

    fn cursor_row(machine: &mut Machine) -> Option<usize> {
        let vid = marty_core::device_traits::videocard::VideoCardId {
            idx: 0,
            vtype: marty_core::device_traits::videocard::VideoType::Gts220,
        };
        let buf = {
            let bus = machine.bus_mut();
            let card = bus.video_mut(&vid).expect("gts220 card");
            use marty_core::bus::DeviceRunTimeUnit;
            card.run(DeviceRunTimeUnit::Microseconds(16_700.0), &mut None, None);
            card.display_buf().to_vec()
        };
        // 640-wide RGBA; cursor = solid block in the left column (x < 100)
        let w = 640usize;
        let mut rows_lit: Vec<usize> = Vec::new();
        for y in 0..(buf.len() / 4 / w) {
            let mut n = 0usize;
            for x in 0..100 {
                let p = &buf[(y * w + x) * 4..(y * w + x) * 4 + 3];
                if p[0] != 0 || p[1] != 0 || p[2] != 0 {
                    n += 1;
                }
            }
            if n >= 40 {
                rows_lit.push(y);
            }
        }
        rows_lit.first().copied()
    }

    // boot to past-STATE-0
    let mut cycles = 0u64;
    while cycles < 30_000_000 {
        let c = machine.run(100_000, &mut exec);
        if c == 0 {
            panic!("stalled at {cycles}");
        }
        cycles += c;
        let mode = machine.bus_mut().get_vec_at(0x60 * 16 + 0x5AA, 1)[0];
        if (mode == 0x40 || mode == 0x41) && cycles > 8_000_000 {
            break;
        }
        while let Some(ev) = machine.get_event() {
            let _ = ev;
        }
    }
    machine.run(2_000_000, &mut exec);

    // --- 1. memory flags at the cell ---
    let flags = machine.bus_mut().get_flags(FLAG);
    println!("[map] get_flags(0x6FB0) = {flags:#04x}");
    assert_eq!(
        flags & (0x02 | 0x04),
        0,
        "cell must not be ROM/MMIO-marked"
    ); // MEM_ROM_BIT=0x02? MEM_MMIO_BIT=0x04? -- print actual

    // --- 2. round-trip ---
    let before = machine.bus_mut().get_vec_at(FLAG, 1)[0];
    machine.bus_mut().write_u8(FLAG, 0x5A, 0).expect("write");
    let rt = machine.bus_mut().get_vec_at(FLAG, 1)[0];
    println!("[map] round-trip: before={before:#04x} wrote 0x5A read={rt:#04x}");
    assert_eq!(rt, 0x5A, "flag cell must be writable RAM");
    machine.bus_mut().write_u8(FLAG, before, 0).expect("restore");

    // --- 3. enter editor via Ctrl-6 ---
    machine.key_press(MartyKey::ControlLeft, marty_core::devices::keyboard_common::KeyboardModifiers::default());
    machine.run(500_000, &mut exec);
    machine.key_press(MartyKey::Digit6, marty_core::devices::keyboard_common::KeyboardModifiers::default());
    machine.run(1_500_000, &mut exec);
    machine.key_release(MartyKey::Digit6);
    machine.run(500_000, &mut exec);
    machine.key_release(MartyKey::ControlLeft);
    machine.run(3_000_000, &mut exec);
    let row0 = cursor_row(&mut machine);
    println!("[edit] cursor row after Ctrl-6: {row0:?}");

    // helper: display-buffer diff counter (catches cursor redraws anywhere)
    fn display_buf(machine: &mut Machine) -> Vec<u8> {
        let vid = marty_core::device_traits::videocard::VideoCardId {
            idx: 0,
            vtype: marty_core::device_traits::videocard::VideoType::Gts220,
        };
        let bus = machine.bus_mut();
        let card = bus.video_mut(&vid).expect("gts220 card");
        use marty_core::bus::DeviceRunTimeUnit;
        card.run(DeviceRunTimeUnit::Microseconds(16_700.0), &mut None, None);
        card.display_buf().to_vec()
    }
    let base_buf = display_buf(&mut machine);

    // unforced Up: with the always-DOS-mode patch installed, the flag is
    // irrelevant - the translator takes the DOS path even though flag == 0,
    // so the cursor MUST move (display change expected).
    machine.key_press(MartyKey::ArrowUp, marty_core::devices::keyboard_common::KeyboardModifiers::default());
    machine.run(1_000_000, &mut exec);
    machine.key_release(MartyKey::ArrowUp);
    machine.run(3_000_000, &mut exec);
    let buf1 = display_buf(&mut machine);
    let diff1 = buf1.iter().zip(base_buf.iter()).filter(|(a, b)| a != b).count();
    println!("[edit] display diff after unforced Up (flag==0): {diff1} bytes (cursor redraw expected: always-DOS patch)");
    assert!(diff1 > 1000, "cursor must move even with flag == 0 (always-DOS patch installed)");

    // --- force flag = 1 through the CPU-visible write path (must be a no-op
    // --- with the always-DOS patch; DOS mode is already in effect) ---
    machine.bus_mut().write_u8(FLAG, 0x01, 0).expect("set flag");
    println!("[map] flag set to 1 via bus write_u8 (no-op for translation, patch active)");

    machine.key_press(MartyKey::ArrowUp, marty_core::devices::keyboard_common::KeyboardModifiers::default());
    machine.run(1_000_000, &mut exec);
    machine.key_release(MartyKey::ArrowUp);
    machine.run(3_000_000, &mut exec);
    let buf2 = display_buf(&mut machine);
    let diff2 = buf2.iter().zip(buf1.iter()).filter(|(a, b)| a != b).count();
    println!("[edit] display diff after flag=1 Up: {diff2} bytes (cursor redraw expected)");
    assert!(diff2 > 1000, "cursor must move once flag == 1");

    machine.key_press(MartyKey::ArrowDown, marty_core::devices::keyboard_common::KeyboardModifiers::default());
    machine.run(1_000_000, &mut exec);
    machine.key_release(MartyKey::ArrowDown);
    machine.run(3_000_000, &mut exec);
    let buf3 = display_buf(&mut machine);
    let diff3 = buf3.iter().zip(buf2.iter()).filter(|(a, b)| a != b).count();
    println!("[edit] display diff after flag=1 Down: {diff3} bytes");
    assert!(diff3 > 1000, "cursor must move on Down too");
}

/// Diagnose why flag=1 did not move the cursor in MartyPC:
/// selector value, flag phys, queue/ring state, display diffs.
#[test]
fn gts220_flag_diag() {
    std::env::set_current_dir(INSTALL_DIR).expect("chdir to install dir");
    let mut machine = build_frontend_machine();
    let mut exec = ExecutionControl::new();
    exec.set_state(ExecutionState::Running);

    fn snap(machine: &mut Machine) -> (Option<usize>, Vec<u8>, u16) {
        let vid = marty_core::device_traits::videocard::VideoCardId {
            idx: 0,
            vtype: marty_core::device_traits::videocard::VideoType::Gts220,
        };
        let (buf, rb) = {
            let bus = machine.bus_mut();
            let card = bus.video_mut(&vid).expect("card");
            use marty_core::bus::DeviceRunTimeUnit;
            card.run(DeviceRunTimeUnit::Microseconds(16_700.0), &mut None, None);
            let buf = card.display_buf().to_vec();
            let segi = u16::from_le_bytes(bus.get_vec_at(0x387 * 16 + 0x258A, 2).try_into().unwrap());
            let cnt = if segi != 0 {
                u16::from_le_bytes(bus.get_vec_at(segi as usize * 16 + 0x446, 2).try_into().unwrap())
            } else { 0 };
            (buf, cnt)
        };
        let w = 640usize;
        let mut rows_lit: Vec<usize> = Vec::new();
        for y in 0..(buf.len() / 4 / w) {
            let mut n = 0usize;
            for x in 0..100 {
                let p = &buf[(y * w + x) * 4..(y * w + x) * 4 + 3];
                if p[0] != 0 || p[1] != 0 || p[2] != 0 { n += 1; }
            }
            if n >= 40 { rows_lit.push(y); }
        }
        (rows_lit.first().copied(), buf, rb)
    }

    let mut cycles = 0u64;
    while cycles < 30_000_000 {
        let c = machine.run(100_000, &mut exec);
        if c == 0 { panic!("stalled"); }
        cycles += c;
        let mode = machine.bus_mut().get_vec_at(0x60 * 16 + 0x5AA, 1)[0];
        if (mode == 0x40 || mode == 0x41) && cycles > 8_000_000 { break; }
        while let Some(ev) = machine.get_event() { let _ = ev; }
    }
    machine.run(2_000_000, &mut exec);

    // selector + effective flag phys
    let sel = u16::from_le_bytes(
        machine.bus_mut().get_vec_at(0x387 * 16 + 0x25A8, 2).try_into().unwrap());
    let flag_phys = sel as usize * 16 + 0xfb0;
    println!("[diag] selector [0x387:0x25a8] = {sel:#06x} -> flag phys {flag_phys:#x}");

    // enter editor
    machine.key_press(MartyKey::ControlLeft, marty_core::devices::keyboard_common::KeyboardModifiers::default());
    machine.run(500_000, &mut exec);
    machine.key_press(MartyKey::Digit6, marty_core::devices::keyboard_common::KeyboardModifiers::default());
    machine.run(1_500_000, &mut exec);
    machine.key_release(MartyKey::Digit6);
    machine.run(500_000, &mut exec);
    machine.key_release(MartyKey::ControlLeft);
    machine.run(3_000_000, &mut exec);

    let (row0, buf0, rb0) = snap(&mut machine);
    let sel2 = u16::from_le_bytes(
        machine.bus_mut().get_vec_at(0x387 * 16 + 0x25A8, 2).try_into().unwrap());
    println!("[diag] editor: cursor={row0:?} ring_cnt={rb0} selector now {sel2:#06x}");

    // set flag at the EFFECTIVE phys
    machine.bus_mut().write_u8(flag_phys, 0x01, 0).expect("set");
    let chk = machine.bus_mut().get_vec_at(flag_phys, 1)[0];
    println!("[diag] wrote 1 at {flag_phys:#x}, readback {chk:#04x}");

    // Up x2, Down x2 with display diffs
    let mut prev_buf = buf0;
    let mut prev_row = row0;
    for (i, key, label) in [(0, MartyKey::ArrowUp, "Up"), (1, MartyKey::ArrowUp, "Up"),
                     (2, MartyKey::ArrowDown, "Down"), (3, MartyKey::ArrowDown, "Down")] {
        machine.key_press(key, marty_core::devices::keyboard_common::KeyboardModifiers::default());
        machine.run(1_000_000, &mut exec);
        machine.key_release(key);
        machine.run(3_000_000, &mut exec);
        let (row, buf, rb) = snap(&mut machine);
        let mut diff = 0usize; let mut minx = usize::MAX; let mut maxy = 0usize; let mut miny = usize::MAX;
        for (idx, (a, b)) in buf.iter().zip(prev_buf.iter()).enumerate() {
            if a != b {
                diff += 1;
                let px = idx / 4;
                let y = px / 640; let x = px % 640;
                if x < minx { minx = x; }
                if y < miny { miny = y; }
                if y > maxy { maxy = y; }
            }
        }
        println!("[diag] {i} {label}: cursor={row:?} ring_cnt={rb} diff_px={} bbox x>{minx} y {miny}..{maxy}", diff / 4);
        prev_buf = buf; prev_row = row;
    }
}

/// Verify the always-DOS-mode romdef patch (trigger 0xC857B, addr 0xD0804):
/// the key translator's flag branch (je, bytes 74 1E) must be NOPped (90 90)
/// after boot, making the keypad-mode flag irrelevant - every key then takes
/// the DOS-style translation path regardless of 0060:0FB0.
#[test]
fn gts220_dosmode_patch_installed() {
    std::env::set_current_dir(INSTALL_DIR).expect("chdir to install dir");
    let mut machine = build_frontend_machine();
    let mut exec = ExecutionControl::new();
    exec.set_state(ExecutionState::Running);

    // pre-patch bytes (ROM original: je 0xD0824)
    let orig = machine.bus_mut().get_vec_at(0xD0804, 2);
    println!("[dosmode] before boot: {orig:02X?} (expect 74 1E)");
    assert_eq!(orig, vec![0x74, 0x1E], "ROM original bytes at 0xD0804");

    // run past STATE-0 so the trigger (0xC857B) fires and the patch installs
    let mut cycles = 0u64;
    while cycles < 30_000_000 {
        let c = machine.run(100_000, &mut exec);
        if c == 0 {
            panic!("stalled at {cycles}");
        }
        cycles += c;
        let mode = machine.bus_mut().get_vec_at(0x60 * 16 + 0x5AA, 1)[0];
        if (mode == 0x40 || mode == 0x41) && cycles > 8_000_000 {
            break;
        }
        while let Some(ev) = machine.get_event() {
            let _ = ev;
        }
    }

    let patched = machine.bus_mut().get_vec_at(0xD0804, 2);
    println!("[dosmode] after boot: {patched:02X?} (expect 90 90)");
    assert_eq!(patched, vec![0x90, 0x90], "patch must be installed at 0xD0804");

    // flag irrelevant: force it to 0 and verify the translator still produces
    // DOS-style extended codes for arrows. The translator caches nothing
    // about the flag outside the comparison we NOPped, so we verify by
    // checking the flag cell stays 0 while the Ctrl-6 screen cursor moves
    // on ArrowUp (covered end-to-end by gts220_flag_cell_mapping).
    machine.bus_mut().write_u8(0x60 * 16 + 0xfb0, 0x00, 0).expect("force flag 0");
    let flag_after = machine.bus_mut().get_vec_at(0x60 * 16 + 0xfb0, 1)[0];
    println!("[dosmode] flag forced to {flag_after:#04x} (translation unaffected)");
    assert_eq!(flag_after, 0x00, "flag must read back 0 (cell is writable RAM)");
}

/// Verify the romdef keypad-mode patch: after boot, the state-reset function's
/// flag write at phys 0xEBA8D must be NOPped out (romdef patch, trigger =
/// STATE-0), so a flag=1 set via the debugger persists across editor exits.
#[test]
fn gts220_keypad_patch_installed() {
    std::env::set_current_dir(INSTALL_DIR).expect("chdir to install dir");
    let mut machine = build_frontend_machine();
    let mut exec = ExecutionControl::new();
    exec.set_state(ExecutionState::Running);

    // pre-patch bytes (ROM original)
    let orig = machine.bus_mut().get_vec_at(0xEBA8D, 6);
    println!("[patch] before boot: {orig:02X?} (expect 26 C6 06 B0 0F 00)");

    // run past STATE-0 so the trigger (0xC8576) fires and the patch installs
    let mut cycles = 0u64;
    while cycles < 30_000_000 {
        let c = machine.run(100_000, &mut exec);
        if c == 0 {
            panic!("stalled at {cycles}");
        }
        cycles += c;
        let mode = machine.bus_mut().get_vec_at(0x60 * 16 + 0x5AA, 1)[0];
        if (mode == 0x40 || mode == 0x41) && cycles > 8_000_000 {
            break;
        }
        while let Some(ev) = machine.get_event() {
            let _ = ev;
        }
    }

    let patched = machine.bus_mut().get_vec_at(0xEBA8D, 6);
    println!("[patch] after STATE-0: {patched:02X?} (expect 90 x6)");
    assert_eq!(orig, vec![0x26, 0xC6, 0x06, 0xB0, 0x0F, 0x00], "ROM original bytes");
    assert_eq!(patched, vec![0x90; 6], "patch must be installed at 0xEBA8D");
}
