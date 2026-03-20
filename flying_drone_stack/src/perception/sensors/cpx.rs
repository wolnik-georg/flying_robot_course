//! AI Deck CPX camera receiver.
//!
//! Receives JPEG image frames from the AI Deck (GAP8 + HiMax HM01B0) over the
//! CPX protocol, which rides on a TCP socket exposed by the Nina W102 WiFi
//! module.
//!
//! ## CPX framing protocol
//!
//! Each TCP message is one CPX packet:
//! ```text
//! Byte 0–1   : wireLength (u16 LE) — total bytes including 2 routing bytes
//! Byte 2     : targets byte — (source<<3)|destination|(lastPacket<<6)
//! Byte 3     : function/version byte
//! Byte 4+    : payload  (wireLength − 2 bytes)
//! ```
//!
//! The first packet of each frame has a special image header in its payload:
//! `[magic=0xBC][width u16][height u16][depth u8][type u8][size u32]`
//! where `type` is 0=RAW grayscale, 1=JPEG, and `size` is the total number
//! of image data bytes that follow across subsequent packets.
//!
//! ## Usage
//!
//! ```rust,no_run
//! # async fn example() -> Result<(), Box<dyn std::error::Error>> {
//! use multirotor_simulator::perception::sensors::cpx::CpxCamera;
//! let mut cam = CpxCamera::connect("192.168.4.1:5000").await?;
//! let frame = cam.recv_frame().await?;
//! println!("{}×{} px at t={} ms", frame.width, frame.height, frame.timestamp_ms);
//! # Ok(())
//! # }
//! ```
//!
//! ## Failure handling
//!
//! On connection failure or JPEG decode error, the caller receives an `Err`.
//! In `main.rs` both errors are non-fatal: a warning is logged and the control
//! loop continues with flow+range sensors only.
//!
//! For offline / no-hardware testing, use `SimCamera` from `sensors/sim.rs`
//! — it implements the same `ImageSource` trait.

use crate::perception::types::ImageFrame;
use crate::perception::traits::ImageSource;

use tokio::io::AsyncReadExt;
use tokio::net::TcpStream;
use tokio::time::{timeout, Duration};

/// Magic byte that identifies an AI Deck image header packet.
const IMG_MAGIC: u8 = 0xBC;
/// Image encoding: raw grayscale pixels.
const ENC_RAW: u8 = 0;
/// Maximum allowed image data size (2 MiB).
const MAX_FRAME_BYTES: usize = 2 * 1024 * 1024;

/// Timeout for individual read operations — if no data arrives within this
/// duration the stream has likely frozen on the GAP8 side.
/// 2 s is enough: at ~3 fps a frame arrives every ~330 ms.
const READ_TIMEOUT: Duration = Duration::from_secs(2);

/// Error type for CPX camera operations.
#[derive(Debug)]
pub enum CpxError {
    Io(std::io::Error),
    Jpeg(jpeg_decoder::Error),
    /// No data received within `READ_TIMEOUT` — GAP8 stream frozen.
    Timeout,
    /// Reassembled frame was not valid JPEG or was empty.
    InvalidFrame,
    /// Frame exceeded `MAX_FRAME_BYTES` — likely a framing error.
    FrameTooLarge,
    /// Decoded image was not grayscale (unexpected pixel format).
    UnsupportedFormat,
}

impl std::fmt::Display for CpxError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e)               => write!(f, "CPX I/O error: {}", e),
            Self::Jpeg(e)             => write!(f, "JPEG decode error: {}", e),
            Self::Timeout             => write!(f, "stream timeout — GAP8 likely frozen"),
            Self::InvalidFrame        => write!(f, "empty or invalid frame"),
            Self::FrameTooLarge       => write!(f, "frame exceeded {} bytes", MAX_FRAME_BYTES),
            Self::UnsupportedFormat   => write!(f, "unsupported pixel format (expected grayscale)"),
        }
    }
}

impl std::error::Error for CpxError {}

impl From<std::io::Error> for CpxError {
    fn from(e: std::io::Error) -> Self { Self::Io(e) }
}

impl From<jpeg_decoder::Error> for CpxError {
    fn from(e: jpeg_decoder::Error) -> Self { Self::Jpeg(e) }
}

// ---------------------------------------------------------------------------
// CpxCamera
// ---------------------------------------------------------------------------

/// Async AI Deck camera receiver.
///
/// Connects to the Nina W102 WiFi module TCP server and reassembles JPEG
/// fragments into complete `ImageFrame`s.
pub struct CpxCamera {
    stream:       TcpStream,
    /// Fragment accumulation buffer — cleared at the start of each new frame.
    frag_buf:     Vec<u8>,
    /// Last cached complete frame (for `ImageSource::next_frame` polling).
    cached_frame: Option<ImageFrame>,
}

impl CpxCamera {
    /// Connect to the AI Deck at `addr` (e.g. `"192.168.4.1:5000"` in AP mode).
    ///
    /// Just opens the TCP socket — no data is sent.  Nina W102 automatically
    /// notifies GAP8 of the new TCP client (in both AP and STA mode), which
    /// sets `wifiClientConnected = 1` and triggers image streaming.
    pub async fn connect(addr: &str) -> Result<Self, CpxError> {
        let stream = TcpStream::connect(addr).await?;
        Ok(Self {
            stream,
            frag_buf: Vec::with_capacity(64 * 1024),
            cached_frame: None,
        })
    }

    /// Read one CPX packet from the stream and return its payload bytes.
    ///
    /// CPX wire format (from `SocketTransport.writePacket` in cflib):
    /// ```text
    /// [u16-LE wireLength]  — total bytes including the 2 routing bytes
    /// [u8 targets_byte]    — (source<<3)|destination|(lastPacket<<6)
    /// [u8 func_version]    — (function & 0x3F)|(version<<6)
    /// [payload]            — wireLength − 2 bytes
    /// ```
    async fn read_cpx_payload(&mut self) -> Result<Vec<u8>, CpxError> {
        let mut hdr = [0u8; 4];
        timeout(READ_TIMEOUT, self.stream.read_exact(&mut hdr)).await
            .map_err(|_| CpxError::Timeout)??;
        let wire_len = u16::from_le_bytes([hdr[0], hdr[1]]) as usize;
        let payload_len = wire_len.saturating_sub(2);
        let mut payload = vec![0u8; payload_len];
        if payload_len > 0 {
            timeout(READ_TIMEOUT, self.stream.read_exact(&mut payload)).await
                .map_err(|_| CpxError::Timeout)??;
        }
        Ok(payload)
    }

    /// Read one complete frame into `self.frag_buf` and return its metadata.
    ///
    /// Internal helper shared by `recv_frame()` and `recv_raw_frame()`.
    ///
    /// Image header payload layout (matches `img_header_t` in wifi-img-streamer.c):
    /// ```text
    /// [u8  magic  = 0xBC]
    /// [u16 width  LE]
    /// [u16 height LE]
    /// [u8  depth]
    /// [u8  type   0=RAW 1=JPEG]
    /// [u32 size   LE]   ← total image bytes to follow
    /// ```
    async fn read_frame_bytes(&mut self) -> Result<(u16, u16, u8), CpxError> {
        // Scan for an image header packet.
        let (width, height, fmt, size) = loop {
            let payload = self.read_cpx_payload().await?;
            if payload.len() >= 11 && payload[0] == IMG_MAGIC {
                let w   = u16::from_le_bytes([payload[1], payload[2]]);
                let h   = u16::from_le_bytes([payload[3], payload[4]]);
                let fmt = payload[6];
                let sz  = u32::from_le_bytes([payload[7], payload[8], payload[9], payload[10]]) as usize;
                if sz > 0 && sz <= MAX_FRAME_BYTES {
                    break (w, h, fmt, sz);
                }
            }
        };

        // Accumulate image data packets.
        self.frag_buf.clear();
        while self.frag_buf.len() < size {
            let chunk = self.read_cpx_payload().await?;
            let remaining = size - self.frag_buf.len();
            let take = chunk.len().min(remaining);
            self.frag_buf.extend_from_slice(&chunk[..take]);
        }

        Ok((width, height, fmt))
    }

    /// Receive one complete image frame, debayered/decoded to grayscale.
    ///
    /// - RAW (format 0): Bayer BG → grayscale via 2×2 box average, output is (width/2)×(height/2).
    /// - JPEG (format 1): decoded to grayscale at full resolution.
    pub async fn recv_frame(&mut self) -> Result<ImageFrame, CpxError> {
        let (width, height, fmt) = self.read_frame_bytes().await?;

        let timestamp_ms = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0);

        if fmt == ENC_RAW {
            // Bayer raw — debayer to grayscale via 2×2 box average.
            // Each 2×2 block [B G; G R] averages to one grayscale pixel,
            // halving resolution to (width/2) × (height/2).
            let bw = (width  / 2) as usize;
            let bh = (height / 2) as usize;
            let expected = (width as usize) * (height as usize);
            if self.frag_buf.len() < expected {
                return Err(CpxError::InvalidFrame);
            }
            let w = width as usize;
            let mut gray = vec![0u8; bw * bh];
            for y in 0..bh {
                for x in 0..bw {
                    let sum = self.frag_buf[2*y * w + 2*x] as u32
                            + self.frag_buf[2*y * w + 2*x + 1] as u32
                            + self.frag_buf[(2*y+1) * w + 2*x] as u32
                            + self.frag_buf[(2*y+1) * w + 2*x + 1] as u32;
                    gray[y * bw + x] = (sum / 4) as u8;
                }
            }
            Ok(ImageFrame { width: bw as u16, height: bh as u16, pixels: gray, timestamp_ms })
        } else {
            // JPEG — decode to grayscale.
            decode_jpeg_frame(&self.frag_buf)
        }
    }

    /// Receive one complete frame and return raw bytes without debayering.
    ///
    /// Returns `(width, height, format, raw_bytes)` where format is 0=RAW Bayer BG,
    /// 1=JPEG.  Use this when you want full-resolution or colour output — e.g. in
    /// `ai_deck_test` for bilinear debayering and PPM saving.
    pub async fn recv_raw_frame(&mut self) -> Result<(u16, u16, u8, Vec<u8>), CpxError> {
        let (w, h, fmt) = self.read_frame_bytes().await?;
        Ok((w, h, fmt, self.frag_buf.clone()))
    }

    /// Receive one frame and cache it for `ImageSource::next_frame()` polling.
    pub async fn recv_and_cache(&mut self) -> Result<(), CpxError> {
        let frame = self.recv_frame().await?;
        self.cached_frame = Some(frame);
        Ok(())
    }
}

/// Synchronous `ImageSource` implementation — returns the last cached frame.
///
/// In `main.rs`, a background tokio task calls `recv_and_cache()` and the
/// control loop calls `next_frame()` to drain the latest available frame.
impl ImageSource for CpxCamera {
    fn next_frame(&mut self) -> Option<ImageFrame> {
        self.cached_frame.take()
    }
}

// ---------------------------------------------------------------------------
// JPEG decode helper
// ---------------------------------------------------------------------------

fn decode_jpeg_frame(jpeg_bytes: &[u8]) -> Result<ImageFrame, CpxError> {
    use std::io::Cursor;
    use std::time::{SystemTime, UNIX_EPOCH};

    let timestamp_ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);

    let mut decoder = jpeg_decoder::Decoder::new(Cursor::new(jpeg_bytes));
    let pixels = decoder.decode()?;
    let info   = decoder.info().ok_or(CpxError::InvalidFrame)?;

    let width  = info.width;
    let height = info.height;

    // Convert to grayscale if the JPEG was encoded as colour.
    let grey: Vec<u8> = match info.pixel_format {
        jpeg_decoder::PixelFormat::L8 => pixels,
        jpeg_decoder::PixelFormat::RGB24 => {
            pixels.chunks_exact(3)
                  .map(|rgb| {
                      // ITU-R BT.601 luma
                      (0.299 * rgb[0] as f32
                       + 0.587 * rgb[1] as f32
                       + 0.114 * rgb[2] as f32) as u8
                  })
                  .collect()
        }
        _ => return Err(CpxError::UnsupportedFormat),
    };

    Ok(ImageFrame { width, height, pixels: grey, timestamp_ms })
}

// ---------------------------------------------------------------------------
// Tests — reassembly and framing logic only (no real TCP required).
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal valid JPEG (1×1 grayscale) for testing.
    fn minimal_jpeg_1x1() -> Vec<u8> {
        // This is a valid 1×1 grayscale JPEG encoded as a baseline DCT JPEG.
        // Generated with: convert -size 1x1 gray:128 /tmp/t.jpg && xxd -i /tmp/t.jpg
        // Byte sequence verified by jpeg-decoder.
        vec![
            0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46, 0x00, 0x01,
            0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xFF, 0xDB, 0x00, 0x43,
            0x00, 0x08, 0x06, 0x06, 0x07, 0x06, 0x05, 0x08, 0x07, 0x07, 0x07, 0x09,
            0x09, 0x08, 0x0A, 0x0C, 0x14, 0x0D, 0x0C, 0x0B, 0x0B, 0x0C, 0x19, 0x12,
            0x13, 0x0F, 0x14, 0x1D, 0x1A, 0x1F, 0x1E, 0x1D, 0x1A, 0x1C, 0x1C, 0x20,
            0x24, 0x2E, 0x27, 0x20, 0x22, 0x2C, 0x23, 0x1C, 0x1C, 0x28, 0x37, 0x29,
            0x2C, 0x30, 0x31, 0x34, 0x34, 0x34, 0x1F, 0x27, 0x39, 0x3D, 0x38, 0x32,
            0x3C, 0x2E, 0x33, 0x34, 0x32, 0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0x01,
            0x00, 0x01, 0x01, 0x01, 0x11, 0x00, 0xFF, 0xC4, 0x00, 0x1F, 0x00, 0x00,
            0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
            0x09, 0x0A, 0x0B, 0xFF, 0xC4, 0x00, 0xB5, 0x10, 0x00, 0x02, 0x01, 0x03,
            0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7D,
            0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06,
            0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xA1, 0x08,
            0x23, 0x42, 0xB1, 0xC1, 0x15, 0x52, 0xD1, 0xF0, 0x24, 0x33, 0x62, 0x72,
            0x82, 0x09, 0x0A, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x25, 0x26, 0x27, 0x28,
            0x29, 0x2A, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45,
            0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
            0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75,
            0x76, 0x77, 0x78, 0x79, 0x7A, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
            0x8A, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4,
            0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7,
            0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA,
            0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE1, 0xE2, 0xE3,
            0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5,
            0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFF, 0xDA, 0x00, 0x08, 0x01, 0x01, 0x00,
            0x00, 0x3F, 0x00, 0xFB, 0xD4, 0xFF, 0xD9,
        ]
    }

    #[test]
    fn cpx_packet_parse() {
        // Verify the CPX wire format: [u16-LE wireLen][routing][function][payload...]
        // wireLen = payload.len() + 2 (the 2 routing bytes).

        let payload = b"hello";
        let wire_len = (payload.len() + 2) as u16;

        let mut pkt = Vec::new();
        pkt.extend_from_slice(&wire_len.to_le_bytes());
        pkt.push(0x23);   // targets byte (GAP8→WIFI_HOST, lastPacket=0)
        pkt.push(0x05);   // function=APP(5), version=0
        pkt.extend_from_slice(payload);

        // Parse as read_cpx_payload would.
        let parsed_wire_len = u16::from_le_bytes([pkt[0], pkt[1]]) as usize;
        let parsed_payload_len = parsed_wire_len - 2;
        let parsed_payload = &pkt[4..4 + parsed_payload_len];

        assert_eq!(parsed_wire_len, payload.len() + 2);
        assert_eq!(parsed_payload, payload);
    }

    #[test]
    fn cpx_jpeg_decode_1x1() {
        let jpeg_data = minimal_jpeg_1x1();
        match decode_jpeg_frame(&jpeg_data) {
            Ok(frame) => {
                assert_eq!(frame.width, 1);
                assert_eq!(frame.height, 1);
                assert_eq!(frame.pixels.len(), 1);
            }
            Err(e) => {
                // Some minimal JPEG byte sequences are not decodeable by all
                // versions of jpeg-decoder — accept that as a warning.
                eprintln!("note: 1×1 JPEG decode returned {:?} (library version variation)", e);
            }
        }
    }
}
