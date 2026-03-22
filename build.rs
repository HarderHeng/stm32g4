//! Build script for STM32G4 Bootloader and APP
//!
//! Generates memory.x with the correct configuration based on the "bootloader" feature.

fn main() {
    // Get the output directory
    let out_dir = std::env::var("OUT_DIR").unwrap();

    // Check if bootloader feature is enabled
    let is_bootloader = std::env::var("CARGO_FEATURE_BOOTLOADER").is_ok();

    // Select the appropriate memory file content
    let memory_content = if is_bootloader {
        include_str!("memory-bootloader.x")
    } else {
        include_str!("memory-app.x")
    };

    // Write memory.x with the correct content
    let memory_x_path = std::path::Path::new(&out_dir).join("memory.x");
    std::fs::write(&memory_x_path, memory_content).expect("Failed to write memory.x");

    // Tell Cargo to link with the standard link script
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");

    // Add defmt.x for APP only
    if !is_bootloader {
        println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    }

    // Add OUT_DIR to the linker search path
    println!("cargo:rustc-link-search=native={}", out_dir);

    // Re-run if files change
    println!("cargo:rerun-if-changed=memory-bootloader.x");
    println!("cargo:rerun-if-changed=memory-app.x");
}