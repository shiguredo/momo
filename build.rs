fn main() {
    // git のコミットハッシュをコンパイル時に埋め込む
    let commit = std::process::Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .and_then(|o| {
            if o.status.success() {
                String::from_utf8(o.stdout)
                    .ok()
                    .map(|s| s.trim().to_string())
            } else {
                None
            }
        })
        .unwrap_or_else(|| "unknown".to_string());

    println!("cargo:rustc-env=MOMO_COMMIT_SHORT={commit}");

    // 有効な feature をビルドフラグとして埋め込む
    let mut flags = Vec::new();
    if std::env::var("CARGO_FEATURE_AYAME").is_ok() {
        flags.push("ayame");
    }
    if std::env::var("CARGO_FEATURE_SORA").is_ok() {
        flags.push("sora");
    }
    if std::env::var("CARGO_FEATURE_RASPBERRYPI").is_ok() {
        flags.push("raspberrypi");
    }
    if std::env::var("CARGO_FEATURE_PREVIEW").is_ok() {
        flags.push("preview");
    }
    println!("cargo:rustc-env=MOMO_BUILD_FLAGS={}", flags.join(", "));

    // git の変更で毎回再ビルドしない
    println!("cargo:rerun-if-changed=.git/HEAD");
    println!("cargo:rerun-if-changed=.git/refs");
}
