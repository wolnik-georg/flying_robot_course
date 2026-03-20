//! AI Deck camera validation tool.
//!
//! Connects to the AI Deck CPX WiFi stream, receives N frames, runs FAST-9
//! feature detection on each, and saves images so you can visually confirm
//! the camera is working.
//!
//! ## Usage
//!
//!   cargo run --release --bin ai_deck_test -- [--addr IP:PORT] [--frames N] [--out PATH]
//!
//! Defaults:
//!   --addr   192.168.4.1:5000   (AI Deck AP mode default)
//!   --frames 10
//!   --out    results/data/ai_deck_frame0.pgm
//!
//! ## Output files
//!
//!   results/data/ai_deck_frame0.pgm          — grayscale (162×122), feature-detection input
//!   results/data/ai_deck_frame0_color.ppm    — full-resolution colour (324×244), for visual check
//!
//! With --save-all, every frame is saved:
//!   results/data/frame_0000.pgm / frame_0000_color.ppm  (and so on)
//!
//! ## What "pass" looks like
//!
//!   Connected to 192.168.4.1:5000
//!   Frame 0: 324×244 px (raw), 41 features detected
//!   Frame 1: 324×244 px (raw), 38 features detected
//!   ...
//!   AI Deck validation PASSED (mean features = 39)
//!
//! ## What "fail" looks like
//!
//!   Error: connection refused (192.168.4.1:5000)  → AI Deck not in AP mode / not powered
//!   OR
//!   Frame 0: 324×244 px (raw), 0 features detected  → dark image, lens cap, or bad threshold

use multirotor_simulator::perception::sensors::cpx::CpxCamera;
use multirotor_simulator::perception::processing::features::detect_features;
use multirotor_simulator::perception::types::ImageFrame;
use std::fs;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();

    let addr = args.iter()
        .position(|a| a == "--addr")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
        .unwrap_or("192.168.4.1:5000");

    let n_frames: usize = args.iter()
        .position(|a| a == "--frames")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(10);

    let out_path = args.iter()
        .position(|a| a == "--out")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
        .unwrap_or("results/data/ai_deck_frame0.pgm");

    let save_all = args.iter().any(|a| a == "--save-all");

    println!("AI Deck validation");
    println!("  addr    : {}", addr);
    println!("  frames  : {}", n_frames);
    println!("  save to : {}", if save_all { "results/data/frame_NNNN.jpg (JPEG) or .pgm+.ppm (raw)" } else { out_path });
    println!();
    println!("Connecting to {} ...", addr);
    println!("(Make sure laptop is connected to the AI Deck WiFi AP first — ping 192.168.4.1)\n");

    let mut cam = tokio::time::timeout(
        std::time::Duration::from_secs(30),
        CpxCamera::connect(addr),
    ).await
    .map_err(|_| format!("Timed out after 30s — is the laptop connected to the AI Deck WiFi? (ping {})", addr))?
    .map_err(|e| format!("Connection failed: {} — is the AI Deck powered and firmware flashed?", e))?;
    println!("Connected!\n");

    let mut total_features = 0u64;
    let mut first_frame_saved = false;

    use multirotor_simulator::perception::sensors::cpx::CpxError;
    let mut i = 0;
    while i < n_frames {
        // Receive raw frame (Bayer or JPEG) for full-resolution colour saving.
        let (raw_w, raw_h, fmt, raw_bytes) = match cam.recv_raw_frame().await {
            Ok(f) => f,
            Err(CpxError::Timeout) => {
                println!("Frame {:2}: stream frozen.", i);
                cam = reconnect(cam, addr).await?;
                continue;
            }
            Err(e) => return Err(e.into()),
        };

        let fmt_name = if fmt == 0 { "raw" } else { "jpeg" };

        // Decode frame → grayscale ImageFrame (FAST-9 input) + native RGB bytes (PPM output).
        let (gray_frame, jpeg_rgb) = if fmt == 0 {
            let gf = bayer_to_gray_box(&raw_bytes, raw_w as usize, raw_h as usize);
            (gf, Vec::new()) // RGB for RAW built on demand below (bilinear debayer)
        } else {
            decode_jpeg(&raw_bytes)?
        };

        let feats = detect_features(&gray_frame, 20);
        let n = feats.len();
        total_features += n as u64;

        println!("Frame {:2}: {}×{} px ({}), {} features detected  {}",
            i, raw_w, raw_h, fmt_name, n,
            if n == 0 { "<-- WARNING: zero features" } else { "" });

        fs::create_dir_all("results/data")?;

        if save_all {
            if fmt == 0 {
                // Raw Bayer: grayscale PGM + bilinear-debayered colour PPM.
                let pgm_path = format!("results/data/frame_{:04}.pgm", i);
                write_pgm(&pgm_path, &gray_frame.pixels, gray_frame.width as usize, gray_frame.height as usize)?;
                let ppm_path = format!("results/data/frame_{:04}_color.ppm", i);
                let rgb = bayer_bg_to_rgb_bilinear(&raw_bytes, raw_w as usize, raw_h as usize);
                write_ppm(&ppm_path, &rgb, raw_w as usize, raw_h as usize)?;
            } else {
                // JPEG: save raw bytes + native-colour PPM (grey→grey, colour→colour).
                let jpg_path = format!("results/data/frame_{:04}.jpg", i);
                fs::write(&jpg_path, &raw_bytes)?;
                let ppm_path = format!("results/data/frame_{:04}_color.ppm", i);
                write_ppm(&ppm_path, &jpeg_rgb, gray_frame.width as usize, gray_frame.height as usize)?;
            }
            if i == 0 {
                let ext = if fmt == 0 { "pgm + _color.ppm" } else { "jpg + _color.ppm" };
                println!("  -> Saved frame_0000.{}", ext);
                println!("     view: ffplay -f image2 -framerate 3 results/data/frame_%04d_color.ppm");
            }
        } else if !first_frame_saved {
            if fmt == 0 {
                write_pgm(out_path, &gray_frame.pixels, gray_frame.width as usize, gray_frame.height as usize)?;
                let color_path = out_path.replace(".pgm", "_color.ppm");
                let rgb = bayer_bg_to_rgb_bilinear(&raw_bytes, raw_w as usize, raw_h as usize);
                write_ppm(&color_path, &rgb, raw_w as usize, raw_h as usize)?;
                println!("  -> Saved {}", color_path);
            } else {
                let jpg_path  = out_path.replace(".pgm", ".jpg");
                let ppm_path  = out_path.replace(".pgm", "_color.ppm");
                fs::write(&jpg_path, &raw_bytes)?;
                write_ppm(&ppm_path, &jpeg_rgb, gray_frame.width as usize, gray_frame.height as usize)?;
                println!("  -> Saved {}", ppm_path);
                println!("     (open with: eog {})", ppm_path);
            }
            first_frame_saved = true;
        }
        i += 1;
    }

    let mean = total_features as f64 / n_frames as f64;
    println!();
    if mean > 5.0 {
        println!("AI Deck validation PASSED  (mean features = {:.0})", mean);
    } else {
        println!("AI Deck validation FAILED  (mean features = {:.0} — check lighting / lens)", mean);
        std::process::exit(1);
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// Reconnect helper
// ---------------------------------------------------------------------------

/// Drop the old camera, then reconnect — handling two distinct failure modes.
///
/// ## Why ordering matters
/// If we called `connect()` before dropping the old socket, Nina would briefly
/// see two clients simultaneously.  It would send CLIENT_CONNECTED (new), then
/// CLIENT_DISCONNECTED (old), leaving GAP8 with `wifiClientConnected=0` and the
/// stream permanently stuck.  Drop first, then connect.
///
/// ## Two failure modes after a freeze
///
/// **Mode A — TCP busy / TIME_WAIT** (`connect()` times out after ~2 s):
///   Nina accepted our FIN but the server socket is still in TIME_WAIT, or
///   GAP8 is still inside `cpxSendPacketBlocking` and Nina's buffer isn't
///   drained yet.  Fix: wait 5 s then retry — the old connection closes and
///   GAP8 loops back to check `wifiClientConnected`.
///
/// **Mode B — WiFi gone** ("No route to host" / os error 113):
///   Nina's heap overflowed from buffering raw frames while GAP8 kept pushing
///   via UART.  Nina crashed and rebooted, taking the WiFi AP down with it.
///   Nina auto-reboots in ~15–20 s; the laptop OS reconnects to the AP in ~5 s.
///   Fix: wait 8 s between retries so we don't flood the AP during reboot.
///   Total 90 s window covers the worst case.
///
/// With JPEG firmware (the fix), Mode B should never happen — JPEG frames are
/// ~5 KB vs 79 KB raw, so Nina's buffer never overflows.  This function still
/// handles it for robustness.
async fn reconnect(
    old: CpxCamera,
    addr: &str,
) -> Result<CpxCamera, Box<dyn std::error::Error>> {
    use std::time::{Duration, Instant};

    // Close the old socket first so Nina sees exactly one disconnect event.
    drop(old);

    // 2 s lets the TCP FIN propagate through Nina → STM32 → GAP8 (CPX path).
    // Without this pause the next SYN arrives while the old connection is still
    // being torn down, causing Mode A timeouts from the very first attempt.
    tokio::time::sleep(Duration::from_secs(2)).await;

    println!("Reconnecting...  (up to 90 s)");
    let start    = Instant::now();
    let window   = Duration::from_secs(90);
    let mut attempt = 0u32;

    loop {
        let elapsed = start.elapsed();
        if elapsed >= window {
            return Err(format!(
                "Could not reconnect within {}s ({} attempts). \
                 Power-cycle the Crazyflie if this repeats.",
                window.as_secs(), attempt
            ).into());
        }

        attempt += 1;

        // 2 s TCP-connect timeout: a healthy Nina answers the SYN in <100 ms.
        // If it takes longer, the server socket is busy (Mode A).
        match tokio::time::timeout(Duration::from_secs(2), CpxCamera::connect(addr)).await {
            Ok(Ok(cam)) => {
                println!("  Connected after {:.1}s (attempt {}).", elapsed.as_secs_f32(), attempt);
                return Ok(cam);
            }

            // Mode B: WiFi AP is gone — Nina is rebooting.
            Ok(Err(ref e)) if is_no_route(e) => {
                println!("  attempt {}: WiFi down (Nina rebooting) — waiting 8 s...", attempt);
                tokio::time::sleep(Duration::from_secs(8)).await;
            }

            // Mode A or other OS error: server busy / old connection still closing.
            Ok(Err(e)) => {
                println!("  attempt {}: {} — waiting 5 s...", attempt, e);
                tokio::time::sleep(Duration::from_secs(5)).await;
            }

            // TCP connect itself timed out — Mode A (TIME_WAIT or backlog).
            Err(_) => {
                println!("  attempt {}: TCP busy (old connection still closing) — waiting 5 s...", attempt);
                tokio::time::sleep(Duration::from_secs(5)).await;
            }
        }
    }
}

/// True when the error is "no route to host" (os error 113) — Nina's WiFi is down.
fn is_no_route(e: &multirotor_simulator::perception::sensors::cpx::CpxError) -> bool {
    let s = e.to_string();
    s.contains("113") || s.contains("No route") || s.contains("Network unreachable")
}

// ---------------------------------------------------------------------------
// JPEG → (grayscale ImageFrame, native RGB bytes)
// ---------------------------------------------------------------------------

/// Decode a JPEG buffer into a grayscale `ImageFrame` (for FAST-9) and a
/// parallel RGB byte vec (for color PPM output).
///
/// - If the JPEG is already grayscale (L8): RGB is simply R=G=B (natural look).
/// - If the JPEG is colour (RGB24, e.g. YCbCr JPEG): RGB passes through directly.
///
/// Both have the same pixel count (`width × height`); RGB has 3× the bytes.
fn decode_jpeg(jpeg_bytes: &[u8]) -> Result<(ImageFrame, Vec<u8>), Box<dyn std::error::Error>> {
    use std::io::Cursor;
    let mut dec = jpeg_decoder::Decoder::new(Cursor::new(jpeg_bytes));
    let pixels = dec.decode()?;
    let info = dec.info().ok_or("JPEG decode: missing info")?;
    let (gray, rgb): (Vec<u8>, Vec<u8>) = match info.pixel_format {
        jpeg_decoder::PixelFormat::L8 => {
            // Grayscale JPEG (HiMax HM01B0 + GAP8 encoder with flags=0).
            // Expand to RGB so the PPM viewer shows a natural grey image.
            let rgb = pixels.iter().flat_map(|&v| [v, v, v]).collect();
            (pixels, rgb)
        }
        jpeg_decoder::PixelFormat::RGB24 => {
            // Colour JPEG — keep RGB, derive grey via BT.601 luma.
            let gray = pixels.chunks_exact(3)
                .map(|c| ((c[0] as u32 * 299 + c[1] as u32 * 587 + c[2] as u32 * 114) / 1000) as u8)
                .collect();
            (gray, pixels)
        }
        _ => return Err("Unsupported JPEG pixel format".into()),
    };
    let frame = ImageFrame { width: info.width, height: info.height, pixels: gray, timestamp_ms: 0 };
    Ok((frame, rgb))
}

// ---------------------------------------------------------------------------
// Bayer BG → grayscale (2×2 box average, half resolution)
// ---------------------------------------------------------------------------

/// Convert a Bayer BG frame to grayscale by averaging each 2×2 block.
/// Output is (w/2) × (h/2) — used for the FAST-9 feature-detection pipeline.
fn bayer_to_gray_box(bayer: &[u8], w: usize, h: usize) -> ImageFrame {
    let bw = w / 2;
    let bh = h / 2;
    let mut gray = vec![0u8; bw * bh];
    for y in 0..bh {
        for x in 0..bw {
            let sum = bayer[2*y * w + 2*x] as u32
                    + bayer[2*y * w + 2*x + 1] as u32
                    + bayer[(2*y+1) * w + 2*x] as u32
                    + bayer[(2*y+1) * w + 2*x + 1] as u32;
            gray[y * bw + x] = (sum / 4) as u8;
        }
    }
    ImageFrame { width: bw as u16, height: bh as u16, pixels: gray, timestamp_ms: 0 }
}

// ---------------------------------------------------------------------------
// Bayer BG → full-resolution colour (bilinear interpolation)
// ---------------------------------------------------------------------------

/// Convert a Bayer BG frame to full-resolution RGB using bilinear interpolation.
///
/// Bayer BG layout (r=row, c=col):
/// ```text
///   (r even, c even) = Blue
///   (r even, c odd)  = Green  (in Blue row)
///   (r odd,  c even) = Green  (in Red row)
///   (r odd,  c odd)  = Red
/// ```
///
/// Returns a `w × h` byte buffer with 3 bytes per pixel (R, G, B).
/// Open the resulting PPM with `eog`, `feh`, or `display`.
fn bayer_bg_to_rgb_bilinear(bayer: &[u8], w: usize, h: usize) -> Vec<u8> {
    let mut rgb = vec![0u8; w * h * 3];

    // Clamped pixel read helper.
    let get = |r: i32, c: i32| -> u32 {
        let r = r.clamp(0, h as i32 - 1) as usize;
        let c = c.clamp(0, w as i32 - 1) as usize;
        bayer[r * w + c] as u32
    };

    for row in 0..h {
        for col in 0..w {
            let r = row as i32;
            let c = col as i32;

            let (red, green, blue) = if row % 2 == 0 && col % 2 == 0 {
                // Blue pixel — interpolate G from 4-neighbours, R from 4-diagonals.
                let b = get(r, c);
                let g = (get(r, c-1) + get(r, c+1) + get(r-1, c) + get(r+1, c) + 2) / 4;
                let rv = (get(r-1,c-1) + get(r-1,c+1) + get(r+1,c-1) + get(r+1,c+1) + 2) / 4;
                (rv, g, b)
            } else if row % 2 == 0 && col % 2 == 1 {
                // Green pixel in Blue row — interpolate B from H-neighbours, R from V-neighbours.
                let g  = get(r, c);
                let b  = (get(r, c-1) + get(r, c+1) + 1) / 2;
                let rv = (get(r-1, c) + get(r+1, c) + 1) / 2;
                (rv, g, b)
            } else if row % 2 == 1 && col % 2 == 0 {
                // Green pixel in Red row — interpolate R from H-neighbours, B from V-neighbours.
                let g  = get(r, c);
                let rv = (get(r, c-1) + get(r, c+1) + 1) / 2;
                let b  = (get(r-1, c) + get(r+1, c) + 1) / 2;
                (rv, g, b)
            } else {
                // Red pixel — interpolate G from 4-neighbours, B from 4-diagonals.
                let rv = get(r, c);
                let g  = (get(r, c-1) + get(r, c+1) + get(r-1, c) + get(r+1, c) + 2) / 4;
                let b  = (get(r-1,c-1) + get(r-1,c+1) + get(r+1,c-1) + get(r+1,c+1) + 2) / 4;
                (rv, g, b)
            };

            let idx = (row * w + col) * 3;
            rgb[idx]     = red.min(255)   as u8;
            rgb[idx + 1] = green.min(255) as u8;
            rgb[idx + 2] = blue.min(255)  as u8;
        }
    }

    rgb
}

// ---------------------------------------------------------------------------
// Image file writers
// ---------------------------------------------------------------------------

/// Write a raw 8-bit grayscale image as a PGM file (P5 binary format).
fn write_pgm(path: &str, pixels: &[u8], w: usize, h: usize) -> std::io::Result<()> {
    use std::io::Write;
    let mut f = fs::File::create(path)?;
    write!(f, "P5\n{} {}\n255\n", w, h)?;
    f.write_all(pixels)?;
    Ok(())
}

/// Write a 24-bit RGB image as a PPM file (P6 binary format).
/// `rgb` must be `w × h × 3` bytes in row-major R,G,B order.
fn write_ppm(path: &str, rgb: &[u8], w: usize, h: usize) -> std::io::Result<()> {
    use std::io::Write;
    let mut f = fs::File::create(path)?;
    write!(f, "P6\n{} {}\n255\n", w, h)?;
    f.write_all(rgb)?;
    Ok(())
}
