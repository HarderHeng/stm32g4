//! Simple Shell for STM32G4
//!
//! Minimal shell implementation with line editing and command execution.

use embedded_io_async::Write;

/// Maximum command line length
pub const CMD_MAX_LEN: usize = 80;

/// Maximum history entries
pub const HISTORY_COUNT: usize = 4;

/// Shell state
pub struct Shell {
    /// Command line buffer
    line: [u8; CMD_MAX_LEN],
    /// Current line length
    len: usize,
    /// Cursor position
    cursor: usize,
    /// History buffer
    history: [[u8; CMD_MAX_LEN]; HISTORY_COUNT],
    /// History lengths
    history_len: [usize; HISTORY_COUNT],
    /// History count
    history_count: usize,
    /// History head
    history_head: usize,
}

impl Shell {
    /// Create a new shell
    pub const fn new() -> Self {
        Self {
            line: [0; CMD_MAX_LEN],
            len: 0,
            cursor: 0,
            history: [[0; CMD_MAX_LEN]; HISTORY_COUNT],
            history_len: [0; HISTORY_COUNT],
            history_count: 0,
            history_head: 0,
        }
    }

    /// Process input byte, returns Some(()) when line is complete
    pub fn process_byte(&mut self, byte: u8) -> Option<()> {
        match byte {
            // Enter
            b'\r' | b'\n' => {
                if self.len > 0 {
                    // Save to history
                    self.save_history();
                    return Some(());
                }
                None
            }
            // Backspace
            0x7f | b'\x08' => {
                if self.cursor > 0 {
                    // Shift left
                    for i in self.cursor..self.len {
                        self.line[i - 1] = self.line[i];
                    }
                    self.len -= 1;
                    self.cursor -= 1;
                }
                None
            }
            // Control chars
            0x00..=0x1f => None,
            // Regular chars
            _ => {
                if self.len < CMD_MAX_LEN && self.cursor == self.len {
                    self.line[self.len] = byte;
                    self.len += 1;
                    self.cursor += 1;
                }
                None
            }
        }
    }

    fn save_history(&mut self) {
        self.history[self.history_head][..self.len]
            .copy_from_slice(&self.line[..self.len]);
        self.history_len[self.history_head] = self.len;
        self.history_head = (self.history_head + 1) % HISTORY_COUNT;
        if self.history_count < HISTORY_COUNT {
            self.history_count += 1;
        }
    }

    /// Get current line as bytes
    pub fn line(&self) -> &[u8] {
        &self.line[..self.len]
    }

    /// Clear the line after command execution
    pub fn clear(&mut self) {
        self.len = 0;
        self.cursor = 0;
    }

    /// Redraw current line
    pub async fn redraw<W: Write>(&self, tx: &mut W) -> Result<(), W::Error> {
        tx.write(b"\r> ").await?;
        tx.write(&self.line[..self.len]).await?;
        tx.write(b" \x08").await?; // Clear trailing char
        Ok(())
    }
}

/// Print welcome message
pub async fn print_welcome<W: Write>(tx: &mut W) -> Result<(), W::Error> {
    tx.write(b"\r\nSTM32G4 Shell\r\n> ").await?;
    Ok(())
}

/// Execute command
pub async fn execute_command<W: Write>(tx: &mut W, line: &[u8]) -> Result<(), W::Error> {
    // Skip leading spaces
    let line = skip_spaces(line);
    if line.is_empty() {
        return Ok(());
    }

    // Get command name
    let (cmd, args) = split_cmd(line);

    match cmd {
        b"help" => {
            tx.write(b"Commands: help hello clear version echo led system\r\n").await?;
        }
        b"hello" => {
            tx.write(b"Hello").await?;
            if !args.is_empty() {
                tx.write(b" ").await?;
                tx.write(args).await?;
            }
            tx.write(b"!\r\n").await?;
        }
        b"clear" => {
            tx.write(b"\x1b[2J\x1b[H").await?;
        }
        b"version" => {
            tx.write(b"v0.3.0\r\n").await?;
        }
        b"echo" => {
            if !args.is_empty() {
                tx.write(args).await?;
            }
            tx.write(b"\r\n").await?;
        }
        b"led" => {
            match args {
                b"on" => { tx.write(b"LED ON\r\n").await?; }
                b"off" => { tx.write(b"LED OFF\r\n").await?; }
                b"toggle" => { tx.write(b"LED TOGGLE\r\n").await?; }
                _ => { tx.write(b"Usage: led on|off|toggle\r\n").await?; }
            }
        }
        b"system" => {
            if args == b"info" {
                tx.write(b"STM32G431CB 170MHz\r\n").await?;
            } else {
                tx.write(b"Usage: system info\r\n").await?;
            }
        }
        _ => {
            tx.write(b"Unknown: ").await?;
            tx.write(cmd).await?;
            tx.write(b"\r\n").await?;
        }
    }
    Ok(())
}

fn skip_spaces(s: &[u8]) -> &[u8] {
    let i = s.iter().position(|&b| b != b' ').unwrap_or(s.len());
    &s[i..]
}

fn split_cmd(s: &[u8]) -> (&[u8], &[u8]) {
    let end = s.iter().position(|&b| b == b' ').unwrap_or(s.len());
    let cmd = &s[..end];
    let rest = skip_spaces(&s[end..]);
    (cmd, rest)
}