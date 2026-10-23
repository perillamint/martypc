/*
    devices::gts_modem — virtual Hayes-compatible modem for the GTS-220
    modem board (R65C19 + RC2324DPL) interface.

    The real modem board hangs off COM1 (0x3F8, 2400bps, IRQ4). The 8088
    firmware speaks a standard Hayes AT command set to it (RE-verified in
    GTS-220/docs/MODEM-BOOT-FAILURE.md):

        AT&F&W0&W1        factory defaults + store profiles
        ATS0=1S8=1        auto-answer after 1 ring
        ATX3DT1,5,9,0P28  tone dial (X3: no dialtone detection)
        ATA               manual answer
        AT&C1 &D1 S7=255  DCD/DTR options, long connect timeout

    Boot behavior depends only on MSR bits the emulated UART already
    provides (DCD clear -> state 0, RI clear -> dial ready), so the shim is
    only needed once the firmware starts dialing.

    Design: the AT state machine here is pure logic (testable); the TCP
    data link lives behind the "serial" feature and is driven from
    SerialPortController::update() (per-frame). Network connects run on a
    worker thread so a hanging DNS/TCP connect can never stall emulation.

    Dial-string addressing: a dial string containing '.' or ':' is treated
    as a host — "ATDT192.168.1.10:2323" connects to that TCP endpoint
    directly (bare hosts default to port 23, the telnet/BBS convention).
    This lets the firmware's phone book (HITEL*.TEL entries) carry the IP
    as the "phone number". Pure numeric dial strings (real phone numbers
    like "1,5,9,0P28") route to the configured default endpoint instead,
    or report NO CARRIER when none is configured.

    Result codes follow the X3 (extended, no dialtone) format the firmware
    selects: "\r\nCONNECT 2400\r\n", "\r\nNO CARRIER\r\n", etc.
*/

use std::collections::VecDeque;

/// Events surfaced for logging/UI; drained per frame by the controller.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ModemEvent {
    /// The DTE issued a dial command.
    Dial(String),
    /// The DTE hung up (ATH) or the modem dropped the line.
    Hangup,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mode {
    /// Command mode, on hook (idle after boot).
    Command,
    /// A dial is in progress; waiting for the network link.
    Dialing,
    /// Incoming call detected (ring pulse in progress / counting rings).
    Ringing,
    /// Data mode: bytes flow between DTE and the network link.
    Online,
}

pub const DEFAULT_CONNECT_BAUD_MSG: &str = "CONNECT 2400";

/// How long a ring pulse lasts (RI asserted) in milliseconds.
const RING_PULSE_MS: u64 = 1200;
/// Fallback dial timeout when no network endpoint is configured.
const NO_NET_DIAL_TIMEOUT_MS: u64 = 5000;

pub struct AtModem {
    // ---- DTE line state ----
    line: Vec<u8>,
    echo: bool,
    quiet: bool,
    /// S-registers (subset: 0 = rings to auto-answer, 7 = connect timeout s,
    /// 8 = comma pause s). Indexed up to 64 like a real 2400 modem.
    s: [u8; 64],

    mode: Mode,
    dcd: bool,
    /// Ring indicator state exposed to the UART MSR (bit 6).
    ri: bool,
    /// Number of rings counted while waiting for S0 auto-answer.
    ring_count: u8,
    /// Timestamp of the current ring pulse start (RI deassert time).
    ring_start: Option<std::time::Instant>,
    /// Timestamp the dial command was accepted.
    dial_start: Option<std::time::Instant>,
    /// '+++' escape detection in Online mode.
    plus_run: u8,
    plus_last: Option<std::time::Instant>,

    /// Bytes queued for delivery to the DTE (responses + network data).
    to_dte: VecDeque<u8>,
    /// Bytes received from the DTE while Online, awaiting the network.
    to_net: Vec<u8>,
    /// Drained by the controller for logging/UI.
    events: VecDeque<ModemEvent>,

    /// Configured default endpoint ("host:port"), used when the dial string
    /// is a phone number rather than a host.
    net_host: Option<String>,
    /// Resolved target for the CURRENT dial: either the host parsed out of
    /// the dial string (ATDT127.0.0.1:2323) or the configured endpoint.
    dial_target: Option<String>,
    net: NetLink,
}

// ---------------------------------------------------------------------------
// Network link — real TCP behind "serial", inert stub otherwise.
// ---------------------------------------------------------------------------
#[cfg(feature = "serial")]
mod net_link {
    use std::io::{Read, Write};

    pub enum NetPoll {
        Connected,
        Failed,
        Eof,
        Data(Vec<u8>),
    }

    enum NetState {
        Idle,
        Connecting(std::sync::mpsc::Receiver<std::io::Result<std::net::TcpStream>>),
        Online(std::net::TcpStream),
    }

    pub struct NetLink {
        state: NetState,
        wbuf: Vec<u8>,
    }

    impl NetLink {
        pub fn new() -> Self {
            Self {
                state: NetState::Idle,
                wbuf: Vec::new(),
            }
        }

        pub fn connected(&self) -> bool {
            matches!(self.state, NetState::Online(_))
        }

        /// Begin a connect attempt on a worker thread (DNS + TCP can block).
        /// `target` is "host" or "host:port" (port required — resolved by
        /// ToSocketAddrs, so bare IPs need an explicit port or the 23
        /// default applied by the caller).
        pub fn start_connect(&mut self, target: String) -> bool {
            if self.state_matches_connecting() {
                return true;
            }
            let host = target;
            let (tx, rx) = std::sync::mpsc::channel();
            std::thread::spawn(move || {
                use std::net::ToSocketAddrs;
                let result = host
                    .to_socket_addrs()
                    .and_then(|mut it| {
                        it.next()
                            .ok_or_else(|| std::io::Error::other("no address resolved"))
                    })
                    .and_then(|addr| {
                        // Bounded connect so a dead host fails, not hangs.
                        std::net::TcpStream::connect_timeout(
                            &addr,
                            std::time::Duration::from_secs(15),
                        )
                    });
                let _ = tx.send(result);
            });
            self.state = NetState::Connecting(rx);
            true
        }

        fn state_matches_connecting(&self) -> bool {
            matches!(self.state, NetState::Connecting(_))
        }

        /// Drop the link (hangup).
        pub fn disconnect(&mut self) {
            self.state = NetState::Idle;
            self.wbuf.clear();
        }

        /// Queue bytes for the network peer.
        pub fn send(&mut self, data: &[u8]) {
            self.wbuf.extend_from_slice(data);
        }

        /// Per-frame pump: flush writes, poll reads and connect results.
        pub fn pump(&mut self) -> Vec<NetPoll> {
            let mut out = Vec::new();
            match &mut self.state {
                NetState::Idle => {}
                NetState::Connecting(rx) => {
                    if let Ok(result) = rx.try_recv() {
                        match result {
                            Ok(stream) => {
                                let _ = stream.set_nonblocking(true);
                                let _ = stream.set_nodelay(true);
                                self.state = NetState::Online(stream);
                                out.push(NetPoll::Connected);
                            }
                            Err(_) => {
                                self.state = NetState::Idle;
                                out.push(NetPoll::Failed);
                            }
                        }
                    }
                }
                NetState::Online(stream) => {
                    // Flush pending writes.
                    if !self.wbuf.is_empty() {
                        match stream.write(&self.wbuf) {
                            Ok(n) if n == self.wbuf.len() => self.wbuf.clear(),
                            Ok(n) => {
                                self.wbuf.drain(..n);
                            }
                            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                            Err(_) => {
                                self.state = NetState::Idle;
                                out.push(NetPoll::Eof);
                                return out;
                            }
                        }
                    }
                    // Read available data.
                    let mut buf = [0u8; 4096];
                    match stream.read(&mut buf) {
                        Ok(0) => {
                            self.state = NetState::Idle;
                            out.push(NetPoll::Eof);
                        }
                        Ok(n) => out.push(NetPoll::Data(buf[..n].to_vec())),
                        Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                        Err(_) => {
                            self.state = NetState::Idle;
                            out.push(NetPoll::Eof);
                        }
                    }
                }
            }
            out
        }
    }
}
#[cfg(not(feature = "serial"))]
mod net_link {
    //! Inert stub: no TCP on this target; dials always fail.

    pub enum NetPoll {
        Failed,
        Eof,
    }

    pub struct NetLink {
        _host: Option<String>,
    }

    impl NetLink {
        pub fn new() -> Self {
            Self { _host: None }
        }
        pub fn connected(&self) -> bool {
            false
        }
        pub fn start_connect(&mut self, _target: String) -> bool {
            false
        }
        pub fn disconnect(&mut self) {}
        pub fn send(&mut self, _data: &[u8]) {}
        pub fn pump(&mut self) -> Vec<NetPoll> {
            Vec::new()
        }
    }
}
use net_link::NetLink;

impl AtModem {
    pub fn new(net_host: Option<String>) -> Self {
        Self {
            line: Vec::new(),
            echo: true,
            quiet: false,
            s: {
                let mut s = [0u8; 64];
                s[0] = 0; // auto-answer off until firmware sets it
                s[7] = 30; // default connect timeout
                s[8] = 2; // comma pause
                s
            },
            mode: Mode::Command,
            dcd: false,
            ri: false,
            ring_count: 0,
            ring_start: None,
            dial_start: None,
            plus_run: 0,
            plus_last: None,
            to_dte: VecDeque::new(),
            to_net: Vec::new(),
            events: VecDeque::new(),
            net: NetLink::new(),
            net_host,
            dial_target: None,
        }
    }

    pub fn dcd(&self) -> bool {
        self.dcd
    }

    pub fn ri(&self) -> bool {
        self.ri
    }

    /// True while a dial is in progress (debug/diagnostics).
    pub fn is_dialing(&self) -> bool {
        matches!(self.mode, Mode::Dialing)
    }

    pub fn take_events(&mut self) -> Vec<ModemEvent> {
        self.events.drain(..).collect()
    }

    /// Take all bytes the modem wants to deliver to the DTE.
    pub fn take_to_dte(&mut self) -> Vec<u8> {
        self.to_dte.drain(..).collect()
    }

    /// Deliver bytes from the DTE (UART TX path). Call at emulated line pace.
    pub fn dte_tx(&mut self, byte: u8) {
        match self.mode {
            Mode::Online => self.data_mode_tx(byte),
            _ => self.command_mode_tx(byte),
        }
    }

    // -----------------------------------------------------------------------
    // Command mode
    // -----------------------------------------------------------------------
    fn command_mode_tx(&mut self, byte: u8) {
        match byte {
            b'\r' => {
                if self.echo {
                    self.to_dte.push_back(b'\r');
                }
                let line = std::mem::take(&mut self.line);
                self.exec_line(&line);
            }
            0x08 | 0x7F => {
                self.line.pop();
                if self.echo {
                    self.to_dte.push_back(0x08);
                }
            }
            b' ' | b'\t' => {} // ignored in command lines
            _ => {
                if self.line.len() < 80 {
                    self.line.push(byte);
                    if self.echo {
                        self.to_dte.push_back(byte);
                    }
                }
            }
        }
    }

    fn exec_line(&mut self, line: &[u8]) {
        let line_str = String::from_utf8_lossy(line);
        let trimmed = line_str.trim();
        if trimmed.is_empty() {
            return; // bare CR: no response (Hayes behavior)
        }
        let upper = trimmed.to_ascii_uppercase();
        if !upper.starts_with("AT") {
            self.result("ERROR");
            return;
        }
        let rest = upper[2..].as_bytes();
        // Hayes semantics: the whole line executes, then exactly ONE final
        // result code is returned. ERROR aborts the rest of the line.
        // A dial consumes the line and suppresses the immediate result
        // (CONNECT / NO CARRIER arrive later via pump()).
        let mut final_result: Option<&'static str> = Some("OK");
        let mut i = 0usize;
        while i < rest.len() {
            let c = rest[i];
            match c {
                b'E' => {
                    self.echo = self.arg_or(rest, &mut i, 1) != 0;
                }
                b'Q' => {
                    self.quiet = self.arg_or(rest, &mut i, 0) != 0;
                }
                b'V' => {
                    // Verbose result codes: accepted; always verbose.
                    let _ = self.arg_or(rest, &mut i, 1);
                }
                b'X' => {
                    let _ = self.arg_or(rest, &mut i, 4);
                }
                b'H' => {
                    let v = self.arg_or(rest, &mut i, 0);
                    if v == 0 {
                        self.hangup();
                    }
                    // H1 (off-hook) has no line to seize; accepted as no-op.
                }
                b'O' => {
                    if self.net.connected() {
                        self.enter_online();
                    }
                    else {
                        final_result = Some("ERROR");
                        break;
                    }
                }
                b'Z' => {
                    // Reset active profile (our defaults).
                    self.echo = true;
                    self.quiet = false;
                }
                b'A' => {
                    // Manual answer: only meaningful while a call is pending.
                    if self.ri {
                        self.answer_call();
                    }
                    else {
                        final_result = Some("ERROR");
                        break;
                    }
                }
                b'D' => {
                    let number = Self::strip_dial_modifiers(
                        String::from_utf8_lossy(&rest[i + 1..]).to_string(),
                    );
                    self.begin_dial(number);
                    return; // dial consumes the rest of the line, no result now
                }
                b'S' => {
                    // Sn=v / Sn?
                    let mut j = i + 1;
                    let mut reg = 0usize;
                    while j < rest.len() && rest[j].is_ascii_digit() {
                        reg = reg * 10 + (rest[j] - b'0') as usize;
                        j += 1;
                    }
                    if j < rest.len() && rest[j] == b'=' {
                        let mut val = 0usize;
                        let mut k = j + 1;
                        while k < rest.len() && rest[k].is_ascii_digit() {
                            val = val * 10 + (rest[k] - b'0') as usize;
                            k += 1;
                        }
                        if reg < self.s.len() {
                            self.s[reg] = val.min(255) as u8;
                        }
                        // Land the loop on the char after the value (k - 1
                        // because the loop's i += 1 follows).
                        i = k.saturating_sub(1);
                    }
                    else if j < rest.len() && rest[j] == b'?' {
                        let v = self.s.get(reg).copied().unwrap_or(0);
                        self.to_dte.extend(format!("\r\n{v:03}\r\n").into_bytes());
                        i = j;
                    }
                    else {
                        final_result = Some("ERROR");
                        break;
                    }
                }
                b'&' => {
                    // &F &W0/1 &C &D &K &M ... — all accepted.
                    if i + 1 < rest.len() {
                        let sub = rest[i + 1];
                        match sub {
                            b'F' => {
                                // Factory defaults.
                                self.echo = true;
                                self.quiet = false;
                                self.s[0] = 0;
                                self.s[7] = 30;
                                i += 1; // skip the sub-command char
                            }
                            b'W' | b'C' | b'D' | b'K' | b'M' | b'Q' => {
                                // Skip optional numeric argument.
                                let mut k = i + 2;
                                while k < rest.len() && rest[k].is_ascii_digit() {
                                    k += 1;
                                }
                                i = k - 1; // loop's i += 1 lands after arg
                            }
                            _ => {
                                i += 1; // unknown &x: skip sub-command char
                            }
                        }
                    }
                }
                b'M' | b'L' | b'N' | b'P' | b'T' | b'W' | b'Y' | b'B' | b'C' | b'F' | b'G'
                | b'J' | b'K' | b'U' => {
                    // Speaker/line-mode/no-ops: consume optional digit.
                    let _ = self.arg_or(rest, &mut i, 0);
                }
                b'I' => {
                    // Identification: report a generic string.
                    self.to_dte.extend(b"\r\nGTS-220 VIRTUAL MODEM 2400\r\n".iter().copied());
                }
                b';' => {
                    // Dial modifier: return to command mode after dialing.
                }
                _ => {
                    final_result = Some("ERROR");
                    break;
                }
            }
            i += 1;
        }
        if let Some(code) = final_result {
            self.result(code);
        }
    }

    /// Remove leading dial modifiers (T/P pulse-tone selects, W wait, ';').
    fn strip_dial_modifiers(number: String) -> String {
        number
            .trim_start_matches(|c: char| {
                matches!(c.to_ascii_uppercase(), 'T' | 'P' | 'W') || c == ';' || c == ' ' || c == '-'
            })
            .to_string()
    }

    /// Consume an optional numeric argument after the command letter.
    fn arg_or(&self, rest: &[u8], i: &mut usize, default: u8) -> u8 {
        let mut j = *i + 1;
        let mut val: u8 = 0;
        let mut digits = 0;
        while j < rest.len() && rest[j].is_ascii_digit() {
            val = val.saturating_mul(10).saturating_add(rest[j] - b'0');
            j += 1;
            digits += 1;
        }
        *i = j - 1; // caller's i += 1 lands on the next command
        if digits == 0 {
            default
        }
        else {
            val
        }
    }

    fn result(&mut self, code: &str) {
        if !self.quiet {
            self.to_dte
                .extend(format!("\r\n{code}\r\n").into_bytes());
        }
    }

    // -----------------------------------------------------------------------
    // Dial / answer / online
    // -----------------------------------------------------------------------
    /// Classify a dial string: a host dial (contains '.' or ':') resolves to
    /// a TCP target "host[:port]" (port defaults to 23, the telnet/BBS
    /// convention); anything else is a phone number routed to the configured
    /// endpoint, if any.
    fn resolve_dial_target(&self, number: &str) -> Option<String> {
        let n = number.trim();
        if n.is_empty() {
            return None;
        }
        if n.contains('.') || n.contains(':') {
            // Host dial. Uppercased by the AT parser — harmless for DNS.
            if n.contains(':') {
                // host:port — validate the port is numeric.
                let (h, p) = n.rsplit_once(':')?;
                if h.is_empty() || p.is_empty() || !p.bytes().all(|b| b.is_ascii_digit()) {
                    return None;
                }
                Some(n.to_string())
            }
            else {
                // Bare host/IP: default telnet port.
                Some(format!("{n}:23"))
            }
        }
        else {
            // Phone number: use the configured endpoint.
            self.net_host.clone()
        }
    }

    fn begin_dial(&mut self, number: String) {
        log::debug!("GTS modem: dial '{number}'");
        self.events.push_back(ModemEvent::Dial(number.clone()));
        self.mode = Mode::Dialing;
        self.dial_start = Some(std::time::Instant::now());
        self.dial_target = self.resolve_dial_target(&number);
        match self.dial_target.clone() {
            Some(target) => {
                log::info!("GTS modem: dial '{number}' -> connecting to {target}");
                self.net.start_connect(target);
            }
            None => {
                // No endpoint for this dial (phone number, nothing
                // configured): NO CARRIER after a short pause (X3 has no
                // dialtone code). dial_start drives the timeout in pump().
            }
        }
    }

    fn answer_call(&mut self) {
        if self.net.connected() {
            self.send_connect();
        }
        else {
            self.result("NO CARRIER");
        }
        self.ri = false;
        self.ring_start = None;
        self.ring_count = 0;
    }

    fn send_connect(&mut self) {
        self.result(DEFAULT_CONNECT_BAUD_MSG);
        self.dcd = true;
        self.mode = Mode::Online;
        self.plus_run = 0;
    }

    fn enter_online(&mut self) {
        self.mode = Mode::Online;
        self.dcd = true;
    }

    fn hangup(&mut self) {
        if self.mode == Mode::Online || self.mode == Mode::Dialing || self.net.connected() {
            self.events.push_back(ModemEvent::Hangup);
        }
        self.net.disconnect();
        self.dcd = false;
        self.mode = Mode::Command;
        self.dial_start = None;
        self.dial_target = None;
        self.ring_start = None;
        self.ring_count = 0;
        self.to_net.clear();
    }

    /// DTR dropped (the DTE cleared MCR bit 0). With &D1 semantics (the
    /// firmware configures `AT&D1`), dropping DTR terminates the call.
    pub fn dtr_drop(&mut self) {
        if self.mode == Mode::Online {
            self.result("NO CARRIER");
        }
        self.hangup();
    }

    // -----------------------------------------------------------------------
    // Data mode
    // -----------------------------------------------------------------------
    fn data_mode_tx(&mut self, byte: u8) {
        // '+++' escape detection: three pluses with a 1s guard after.
        let now = std::time::Instant::now();
        if byte == b'+' {
            if let Some(last) = self.plus_last {
                if now.duration_since(last).as_millis() > 1000 {
                    self.plus_run = 0;
                }
            }
            self.plus_last = Some(now);
            self.plus_run += 1;
            if self.plus_run >= 3 {
                self.plus_run = 0;
                self.mode = Mode::Command;
                self.result("OK");
            }
            return;
        }
        self.plus_run = 0;
        self.plus_last = None;
        self.to_net.push(byte);
    }

    // -----------------------------------------------------------------------
    // Per-frame pump (called from SerialPortController::update)
    // -----------------------------------------------------------------------

    /// Simulate an incoming ring from the network side. Pulses RI; honors
    /// S0 auto-answer.
    pub fn ring(&mut self) {
        if self.mode != Mode::Command {
            return;
        }
        self.ri = true;
        self.ring_start = Some(std::time::Instant::now());
        self.ring_count += 1;
        self.result("RING");
        let s0 = self.s[0];
        if s0 > 0 && self.ring_count >= s0 {
            // Auto-answer: only if a network peer exists to bridge.
            if self.net.connected() {
                self.answer_call();
            }
        }
    }

    /// Drive timers and the network link. Returns nothing; results are
    /// queued to `to_dte` and state changes surface via dcd()/ri().
    pub fn pump(&mut self) {
        let now = std::time::Instant::now();

        // RI pulse decay.
        if self.ri {
            if let Some(start) = self.ring_start {
                if now.duration_since(start).as_millis() as u64 >= RING_PULSE_MS {
                    self.ri = false;
                }
            }
        }

        // Network pump.
        for poll in self.net.pump() {
            #[cfg(feature = "serial")]
            match poll {
                net_link::NetPoll::Connected => match self.mode {
                    Mode::Dialing => self.send_connect(),
                    _ => {
                        // Unsolicited connection while idle: treat as a
                        // pending incoming call (ring once).
                        self.ring();
                    }
                },
                net_link::NetPoll::Failed => {
                    if self.mode == Mode::Dialing {
                        self.result("NO CARRIER");
                        self.mode = Mode::Command;
                        self.dial_start = None;
                    }
                }
                net_link::NetPoll::Eof => {
                    if self.mode == Mode::Online {
                        self.result("NO CARRIER");
                        self.hangup();
                    }
                }
                net_link::NetPoll::Data(data) => {
                    if self.mode == Mode::Online {
                        self.to_dte.extend(data);
                    }
                    // Data while in command mode is dropped (matches a real
                    // modem ignoring line noise before CONNECT).
                }
            }
            #[cfg(not(feature = "serial"))]
            {
                let _ = poll;
            }
        }

        // Online: push DTE data toward the network.
        if self.mode == Mode::Online && !self.to_net.is_empty() {
            let data = std::mem::take(&mut self.to_net);
            self.net.send(&data);
        }

        // Dial timeout handling.
        if self.mode == Mode::Dialing {
            if let Some(start) = self.dial_start {
                let elapsed = now.duration_since(start).as_millis() as u64;
                if self.dial_target.is_none() && elapsed >= NO_NET_DIAL_TIMEOUT_MS {
                    // No target for this dial (phone number, nothing
                    // configured): line out.
                    self.result("NO CARRIER");
                    self.mode = Mode::Command;
                    self.dial_start = None;
                    self.dial_target = None;
                }
                // With a target the connect thread's own timeout (15s)
                // produces the Failed poll above; S7 caps the wait too.
                let s7_ms = u64::from(self.s[7]).saturating_mul(1000);
                if self.dial_target.is_some() && elapsed >= s7_ms.max(15_000) {
                    self.result("NO CARRIER");
                    self.mode = Mode::Command;
                    self.dial_start = None;
                    self.dial_target = None;
                    self.net.disconnect();
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed(modem: &mut AtModem, cmd: &str) -> String {
        for b in cmd.bytes() {
            modem.dte_tx(b);
        }
        modem.dte_tx(b'\r');
        String::from_utf8(modem.take_to_dte()).unwrap()
    }

    #[test]
    fn basic_at_responses() {
        let mut m = AtModem::new(None);
        // With echo on, the command is echoed then the result follows.
        let resp = feed(&mut m, "AT&F&W0&W1");
        assert!(resp.ends_with("\r\nOK\r\n"), "got {resp:?}");
        let resp = feed(&mut m, "ATS0=1S8=1");
        assert!(resp.ends_with("\r\nOK\r\n"), "got {resp:?}");
        let resp = feed(&mut m, "ATX3");
        assert!(resp.ends_with("\r\nOK\r\n"), "got {resp:?}");
        let resp = feed(&mut m, "AT&C1 &D1 S7=255");
        assert!(resp.ends_with("\r\nOK\r\n"), "got {resp:?}");
    }

    #[test]
    fn s_register_readback() {
        let mut m = AtModem::new(None);
        let resp = feed(&mut m, "ATS0=1");
        assert!(resp.ends_with("\r\nOK\r\n"));
        let resp = feed(&mut m, "ATS0?");
        assert!(resp.contains("001"), "got {resp:?}");
        assert!(resp.ends_with("\r\nOK\r\n"));
    }

    #[test]
    fn dial_without_net_times_out_to_no_carrier() {
        let mut m = AtModem::new(None);
        let resp = feed(&mut m, "ATX3DT1,5,9,0P28");
        assert!(matches!(m.mode, Mode::Dialing));
        assert!(
            m.take_events().contains(&ModemEvent::Dial("1,5,9,0P28".into())),
            "dial event with stripped modifiers, resp={resp:?}"
        );
        // Pump past the no-net timeout.
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(6);
        while std::time::Instant::now() < deadline && m.mode == Mode::Dialing {
            std::thread::sleep(std::time::Duration::from_millis(100));
            m.pump();
        }
        let resp = String::from_utf8(m.take_to_dte()).unwrap();
        assert!(resp.contains("NO CARRIER"), "got {resp:?}");
        assert_eq!(m.mode, Mode::Command);
        assert!(!m.dcd());
    }

    #[test]
    fn ath_hangs_up_and_reports_ok() {
        let mut m = AtModem::new(None);
        let resp = feed(&mut m, "ATH");
        assert!(resp.ends_with("\r\nOK\r\n"));
    }

    #[test]
    fn unknown_command_is_error() {
        let mut m = AtModem::new(None);
        let resp = feed(&mut m, "AT%p%");
        assert!(resp.ends_with("\r\nERROR\r\n"), "got {resp:?}");
    }

    #[test]
    fn echo_off_suppresses_echo() {
        let mut m = AtModem::new(None);
        let _ = feed(&mut m, "ATE0");
        m.take_to_dte();
        let resp = feed(&mut m, "AT");
        assert_eq!(resp, "\r\nOK\r\n", "no echo expected, got {resp:?}");
    }
}
