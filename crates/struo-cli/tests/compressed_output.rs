//! Verify the real CLI's default artifact and explicit external JSON export.
use std::{fs, process::Command};

#[test]
fn default_binary_and_explicit_json_describe_the_same_mapped_netlist() {
    let directory = tempfile::tempdir().unwrap();
    let project = directory.path();
    fs::write(
        project.join("Veryl.toml"),
        r#"
[project]
name = "binary_output_test"
version = "0.1.0"
[build]
sources = ["."]
exclude_std = true
target = { type = "directory", path = "build" }
"#,
    )
    .unwrap();
    fs::write(
        project.join("top.veryl"),
        "module BinaryTest (a: input logic, b: input logic, y: output logic) { assign y = a ^ b; }",
    )
    .unwrap();
    let invoke = |extra: &[&str]| {
        let output = Command::new(env!("CARGO_BIN_EXE_struo"))
            .arg(project)
            .args(["--top", "BinaryTest"])
            .args(extra)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
    };
    invoke(&[]);
    let artifact = project.join("target/struo/BinaryTest.stnet");
    assert!(fs::read(&artifact).unwrap().starts_with(b"STRUOA\x01\n"));
    assert!(!project.join("target/struo/BinaryTest.json").exists());
    let binary: serde_json::Value = struo::target::ecp5::read_artifact(&artifact).unwrap();
    invoke(&["--output-format", "nextpnr-json"]);
    let legacy: serde_json::Value =
        struo::target::ecp5::read_artifact(&project.join("target/struo/BinaryTest.json")).unwrap();
    assert_eq!(binary, legacy);
}
