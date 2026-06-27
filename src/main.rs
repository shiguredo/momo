mod adm;
#[cfg(feature = "ayame")]
mod ayame;
mod codec;
mod error;
mod fake;
#[cfg(feature = "raspberrypi")]
mod libcamera;
mod metrics;
mod openh264;
mod p2p;
#[cfg(feature = "player")]
mod preview;
#[cfg(target_os = "linux")]
mod serial;
#[cfg(feature = "sora")]
mod sora;
#[cfg(feature = "raspberrypi")]
mod v4l2_encoder;
mod webrtc_video;

#[tokio::main]
async fn main() -> noargs::Result<()> {
    let mut args = noargs::raw_args();
    args.metadata_mut().app_name = env!("CARGO_PKG_NAME");
    args.metadata_mut().app_description = env!("CARGO_PKG_DESCRIPTION");

    if noargs::VERSION_FLAG.take(&mut args).is_present() {
        println!(
            "{} {} ({})",
            env!("CARGO_PKG_NAME"),
            env!("CARGO_PKG_VERSION"),
            env!("MOMO_COMMIT_SHORT"),
        );
        println!("WebRTC: Shiguredo-Build {}", shiguredo_webrtc::version(),);
        println!("OpenH264: {} (build)", shiguredo_openh264::BUILD_VERSION,);
        println!("Environment: {}", metrics::build_environment_string());
        let build_flags = env!("MOMO_BUILD_FLAGS");
        if !build_flags.is_empty() {
            println!("Build Flags: {build_flags}");
        }
        return Ok(());
    }

    noargs::HELP_FLAG.take_help(&mut args);

    // ── グローバルフラグ ──
    let no_google_stun = noargs::flag("no-google-stun")
        .doc("Do not use google stun")
        .take(&mut args)
        .is_present();
    let no_video_input_device = noargs::flag("no-video-input-device")
        .doc("Do not use video input device")
        .take(&mut args)
        .is_present();
    let no_audio_device = noargs::flag("no-audio-device")
        .doc("Do not use audio device")
        .take(&mut args)
        .is_present();
    let fake_capture_device = noargs::flag("fake-capture-device")
        .doc("Use fake video capture device instead of real camera")
        .take(&mut args)
        .is_present();
    let list_devices = noargs::flag("list-devices")
        .doc("List available audio and video devices and exit")
        .take(&mut args)
        .is_present();
    let insecure = noargs::flag("insecure")
        .doc("Allow insecure server connections when using SSL")
        .take(&mut args)
        .is_present();
    let fixed_resolution = noargs::flag("fixed-resolution")
        .doc("Maintain video resolution in degradation")
        .take(&mut args)
        .is_present();
    let use_player = noargs::flag("player")
        .doc("Show video using player (if player feature is available)")
        .take(&mut args)
        .is_present();
    let _fullscreen = noargs::flag("fullscreen")
        .doc("Use fullscreen window for videos (if raw player is available)")
        .take(&mut args)
        .is_present();
    let _screen_capture = noargs::flag("screen-capture")
        .doc("Capture screen")
        .take(&mut args)
        .is_present();
    let use_libcamera = noargs::flag("use-libcamera")
        .doc("Use libcamera for video capture (Raspberry Pi only)")
        .take(&mut args)
        .is_present();
    let use_libcamera_native = noargs::flag("use-libcamera-native")
        .doc("Use native DMA-BUF buffer for H.264 encoding (requires --use-libcamera and --use-v4l2-encoder)")
        .take(&mut args)
        .is_present();
    // --libcamera-control KEY=VALUE (複数回指定可能)
    let mut libcamera_controls = Vec::new();
    loop {
        let opt = noargs::opt("libcamera-control")
            .ty("KEY=VALUE")
            .doc("Set libcamera control (repeatable)")
            .take(&mut args);
        if !opt.is_present() {
            break;
        }
        let raw = opt.value().to_string();
        match raw.split_once('=') {
            Some((key, value)) => {
                libcamera_controls.push((key.to_string(), value.to_string()));
            }
            None => {
                return Err(noargs::Error::other(
                    &noargs::raw_args(),
                    format!("--libcamera-control の値は KEY=VALUE 形式で指定してください: {raw}"),
                ));
            }
        }
    }
    let use_v4l2_encoder = noargs::flag("use-v4l2-encoder")
        .doc("Use V4L2 hardware H.264 encoder (Raspberry Pi only)")
        .take(&mut args)
        .is_present();
    let _disable_echo_cancellation = noargs::flag("disable-echo-cancellation")
        .doc("Disable echo cancellation for audio")
        .take(&mut args)
        .is_present();
    let _disable_auto_gain_control = noargs::flag("disable-auto-gain-control")
        .doc("Disable auto gain control for audio")
        .take(&mut args)
        .is_present();
    let _disable_noise_suppression = noargs::flag("disable-noise-suppression")
        .doc("Disable noise suppression for audio")
        .take(&mut args)
        .is_present();
    let _disable_highpass_filter = noargs::flag("disable-highpass-filter")
        .doc("Disable highpass filter for audio")
        .take(&mut args)
        .is_present();
    let metrics_allow_external_ip = noargs::flag("metrics-allow-external-ip")
        .doc("Allow access to Metrics server from external IP")
        .take(&mut args)
        .is_present();
    let force_i420 = noargs::flag("force-i420")
        .doc("Force I420 pixel format for video capture")
        .take(&mut args)
        .is_present();
    let force_yuy2 = noargs::flag("force-yuy2")
        .doc("Force YUY2 pixel format for video capture")
        .take(&mut args)
        .is_present();
    let force_nv12 = noargs::flag("force-nv12")
        .doc("Force NV12 pixel format for video capture")
        .take(&mut args)
        .is_present();
    let video_codec_engines = noargs::flag("video-codec-engines")
        .doc("List available video encoders/decoders")
        .take(&mut args)
        .is_present();

    // ── グローバルオプション ──
    let video_input_device: Option<String> = noargs::opt("video-input-device")
        .ty("DEVICE")
        .doc("Use the video device specified by an index or a name (use the first one if not specified)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let audio_input_device: Option<String> = noargs::opt("audio-input-device")
        .ty("DEVICE")
        .doc("Use the audio input device specified by an index or a name (use the system default if not specified)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let _audio_output_device: Option<String> = noargs::opt("audio-output-device")
        .ty("DEVICE")
        .doc("Use the audio output device specified by an index or a name (use the system default if not specified)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let resolution: String = noargs::opt("resolution")
        .ty("RESOLUTION")
        .doc("Video resolution (one of QVGA, VGA, HD, FHD, 4K, or [WIDTH]x[HEIGHT])")
        .default("VGA")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let (video_width, video_height) =
        parse_resolution(&resolution).map_err(|e| noargs::Error::other(&noargs::raw_args(), e))?;
    let framerate: u32 = noargs::opt("framerate")
        .ty("FRAMERATE")
        .doc("Video framerate")
        .default("30")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let priority: String = noargs::opt("priority")
        .ty("PRIORITY")
        .doc("Specifies the quality that is maintained against video degradation (BALANCE, FRAMERATE, RESOLUTION, DISABLED)")
        .default("FRAMERATE")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let window_width: u32 = noargs::opt("window-width")
        .ty("WIDTH")
        .doc("Window width for videos (if raw player is available)")
        .default("640")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let window_height: u32 = noargs::opt("window-height")
        .ty("HEIGHT")
        .doc("Window height for videos (if raw player is available)")
        .default("480")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let log_level: Option<String> = noargs::opt("log-level")
        .ty("LEVEL")
        .doc("Log severity level threshold (verbose, info, warning, error, none)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let _hw_mjpeg_decoder: Option<String> = noargs::opt("hw-mjpeg-decoder")
        .ty("BOOL")
        .doc("Perform MJPEG decode and video resize by hardware acceleration (only on supported devices)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let metrics_port: i32 = noargs::opt("metrics-port")
        .ty("PORT")
        .doc("Metrics server port number (default: -1)")
        .default("-1")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let client_cert: Option<String> = noargs::opt("client-cert")
        .ty("PATH")
        .doc("Cert file path for client certification (PEM format)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let client_key: Option<String> = noargs::opt("client-key")
        .ty("PATH")
        .doc("Private key file path for client certification (PEM format)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let cacert: Option<String> = noargs::opt("cacert")
        .ty("PATH")
        .doc("CA certificate file path (PEM format)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let proxy_url: Option<String> = noargs::opt("proxy-url")
        .ty("URL")
        .doc("Proxy URL (Sora mode only)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let proxy_username: Option<String> = noargs::opt("proxy-username")
        .ty("USER")
        .doc("Proxy username (Sora mode only)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let proxy_password: Option<String> = noargs::opt("proxy-password")
        .ty("PASS")
        .doc("Proxy password (Sora mode only)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    #[cfg(target_os = "linux")]
    let serial: Option<serial::SerialConfig> = noargs::opt("serial")
        .ty("DEVICE,BAUDRATE")
        .doc("Serial port settings for datachannel passthrough [DEVICE],[BAUDRATE]")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    #[cfg(not(target_os = "linux"))]
    let _serial: Option<String> = noargs::opt("serial")
        .ty("DEVICE,BAUDRATE")
        .doc("Serial port settings for datachannel passthrough [DEVICE],[BAUDRATE]")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let openh264_path: Option<String> = noargs::opt("openh264")
        .ty("PATH")
        .doc("OpenH264 dynamic library path")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;

    // ── ビデオコーデックオプション ──
    let _vp8_encoder: Option<String> = noargs::opt("vp8-encoder")
        .ty("TYPE")
        .doc("VP8 Encoder")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let _vp8_decoder: Option<String> = noargs::opt("vp8-decoder")
        .ty("TYPE")
        .doc("VP8 Decoder")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let _vp9_encoder: Option<String> = noargs::opt("vp9-encoder")
        .ty("TYPE")
        .doc("VP9 Encoder")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let _vp9_decoder: Option<String> = noargs::opt("vp9-decoder")
        .ty("TYPE")
        .doc("VP9 Decoder")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let _av1_encoder: Option<String> = noargs::opt("av1-encoder")
        .ty("TYPE")
        .doc("AV1 Encoder")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let _av1_decoder: Option<String> = noargs::opt("av1-decoder")
        .ty("TYPE")
        .doc("AV1 Decoder")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let h264_encoder_value: Option<String> = noargs::opt("h264-encoder")
        .ty("TYPE")
        .doc("H.264 Encoder")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let h264_decoder_value: Option<String> = noargs::opt("h264-decoder")
        .ty("TYPE")
        .doc("H.264 Decoder")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let h265_encoder_value: Option<String> = noargs::opt("h265-encoder")
        .ty("TYPE")
        .doc("H.265 Encoder")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let h265_decoder_value: Option<String> = noargs::opt("h265-decoder")
        .ty("TYPE")
        .doc("H.265 Decoder")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;

    // ── ピクセルフォーマット強制指定のバリデーション ──
    let force_pixel_format = {
        let count = [force_i420, force_yuy2, force_nv12]
            .iter()
            .filter(|&&v| v)
            .count();
        if count > 1 {
            return Err(noargs::Error::other(
                &noargs::raw_args(),
                "--force-i420, --force-yuy2, --force-nv12 は同時に指定できません",
            ));
        }
        if force_i420 {
            Some(ForcePixelFormat::I420)
        } else if force_yuy2 {
            Some(ForcePixelFormat::Yuy2)
        } else if force_nv12 {
            Some(ForcePixelFormat::Nv12)
        } else {
            None
        }
    };

    // ── DegradationPreference の算出 ──
    // --fixed-resolution は --priority より優先
    let degradation_preference = if fixed_resolution {
        shiguredo_webrtc::DegradationPreference::MaintainResolution
    } else {
        match priority.to_uppercase().as_str() {
            "BALANCE" => shiguredo_webrtc::DegradationPreference::Balanced,
            "FRAMERATE" => shiguredo_webrtc::DegradationPreference::MaintainFramerate,
            "RESOLUTION" => shiguredo_webrtc::DegradationPreference::MaintainResolution,
            "DISABLED" => shiguredo_webrtc::DegradationPreference::Disabled,
            other => {
                return Err(noargs::Error::other(
                    &noargs::raw_args(),
                    format!(
                        "invalid priority: '{other}' (BALANCE, FRAMERATE, RESOLUTION, DISABLED のいずれかを指定してください)"
                    ),
                ));
            }
        }
    };

    // ── クライアント証明書のバリデーション ──
    let client_cert_pem = match (&client_cert, &client_key) {
        (Some(cert_path), Some(key_path)) => {
            let cert = std::fs::read_to_string(cert_path).map_err(|e| {
                noargs::Error::other(
                    &noargs::raw_args(),
                    format!("client-cert file read error: {cert_path}: {e}"),
                )
            })?;
            let key = std::fs::read_to_string(key_path).map_err(|e| {
                noargs::Error::other(
                    &noargs::raw_args(),
                    format!("client-key file read error: {key_path}: {e}"),
                )
            })?;
            Some((cert, key))
        }
        (Some(_), None) => {
            return Err(noargs::Error::other(
                &noargs::raw_args(),
                "--client-cert requires --client-key",
            ));
        }
        (None, Some(_)) => {
            return Err(noargs::Error::other(
                &noargs::raw_args(),
                "--client-key requires --client-cert",
            ));
        }
        (None, None) => None,
    };

    // ── CA 証明書のバリデーション ──
    let ca_cert_pem = match &cacert {
        Some(path) => {
            let pem = std::fs::read_to_string(path).map_err(|e| {
                noargs::Error::other(
                    &noargs::raw_args(),
                    format!("cacert file read error: {path}: {e}"),
                )
            })?;
            Some(pem)
        }
        None => None,
    };

    // ── OpenH264 ライブラリのロード ──
    let openh264_lib = match &openh264_path {
        Some(path) => Some(
            openh264::load_openh264(path)
                .map_err(|e| noargs::Error::other(&noargs::raw_args(), e))?,
        ),
        None => None,
    };

    // ── デバイス一覧 ──
    if list_devices {
        print_devices_json().map_err(|e| noargs::Error::other(&noargs::raw_args(), e))?;
        return Ok(());
    }

    // ── ビデオコーデックエンジン一覧 ──
    if video_codec_engines {
        print_video_codec_engines();
        return Ok(());
    }

    // ── ログ初期化 ──
    init_tracing(log_level.as_deref());

    // ── メトリクスサーバー ──
    let metrics_state = if metrics_port >= 0 {
        let state = metrics::MetricsState::new();
        let s = state.clone();
        let port = metrics_port as u16;
        tokio::spawn(async move {
            if let Err(e) = metrics::run(port, metrics_allow_external_ip, s).await {
                tracing::error!(target: "metrics", error = %e, "metrics server error");
            }
        });
        Some(state)
    } else {
        None
    };

    // ── サブコマンド ──
    let momo_config = MomoConfig {
        no_audio_device,
        no_video_input_device,
        no_google_stun,
        fake_capture_device,
        use_libcamera,
        use_libcamera_native,
        libcamera_controls,
        use_v4l2_encoder,
        openh264_lib,
        h264_encoder: h264_encoder_value,
        h264_decoder: h264_decoder_value,
        h265_encoder: h265_encoder_value,
        h265_decoder: h265_decoder_value,
        video_input_device,
        audio_input_device,
        video_width,
        video_height,
        framerate,
        insecure,
        force_pixel_format,
        client_cert: client_cert_pem,
        ca_cert: ca_cert_pem,
        degradation_preference,
        proxy_url,
        proxy_username,
        proxy_password,
        #[cfg(target_os = "linux")]
        serial,
        use_player,
        window_width,
        window_height,
    };

    if noargs::cmd("p2p")
        .doc("P2P mode for momo development with simple HTTP server")
        .take(&mut args)
        .is_present()
    {
        run_p2p(sub_args(&args), momo_config, metrics_state).await?;
    } else if noargs::cmd("ayame")
        .doc("Mode for working with WebRTC Signaling Server Ayame")
        .take(&mut args)
        .is_present()
    {
        #[cfg(feature = "ayame")]
        {
            run_ayame(sub_args(&args), momo_config, metrics_state).await?;
        }
        #[cfg(not(feature = "ayame"))]
        {
            let _ = (&momo_config, &metrics_state);
            return Err(noargs::Error::other(
                &noargs::raw_args(),
                "ayame mode requires the 'ayame' feature to be enabled",
            ));
        }
    } else if noargs::cmd("sora")
        .doc("Mode for working with WebRTC SFU Sora")
        .take(&mut args)
        .is_present()
    {
        run_sora(sub_args(&args), momo_config, metrics_state).await?;
    } else if let Some(help) = args.finish()? {
        print!("{}", help);
    }

    Ok(())
}

// ─── 共通設定 ──────────────────────────────────────────────────────────────

/// キャプチャピクセルフォーマットの強制指定
#[derive(Debug, Clone, Copy)]
enum ForcePixelFormat {
    I420,
    Yuy2,
    Nv12,
}

impl ForcePixelFormat {
    fn to_pixel_format(self) -> shiguredo_video_device::PixelFormat {
        match self {
            ForcePixelFormat::I420 => shiguredo_video_device::PixelFormat::I420,
            ForcePixelFormat::Yuy2 => shiguredo_video_device::PixelFormat::Yuy2,
            ForcePixelFormat::Nv12 => shiguredo_video_device::PixelFormat::Nv12,
        }
    }
}

#[allow(dead_code)]
struct MomoConfig {
    no_audio_device: bool,
    no_video_input_device: bool,
    no_google_stun: bool,
    fake_capture_device: bool,
    use_libcamera: bool,
    use_libcamera_native: bool,
    libcamera_controls: Vec<(String, String)>,
    use_v4l2_encoder: bool,
    openh264_lib: Option<shiguredo_openh264::Openh264Library>,
    h264_encoder: Option<String>,
    h264_decoder: Option<String>,
    h265_encoder: Option<String>,
    h265_decoder: Option<String>,
    video_input_device: Option<String>,
    audio_input_device: Option<String>,
    video_width: i32,
    video_height: i32,
    framerate: u32,
    insecure: bool,
    force_pixel_format: Option<ForcePixelFormat>,
    /// クライアント証明書 (cert_pem, key_pem)
    client_cert: Option<(String, String)>,
    /// CA 証明書 (PEM)
    ca_cert: Option<String>,
    degradation_preference: shiguredo_webrtc::DegradationPreference,
    /// プロキシ URL (Sora モードのみ利用)
    proxy_url: Option<String>,
    /// プロキシ認証ユーザー名 (Sora モードのみ利用)
    proxy_username: Option<String>,
    /// プロキシ認証パスワード (Sora モードのみ利用)
    proxy_password: Option<String>,
    #[cfg(target_os = "linux")]
    serial: Option<serial::SerialConfig>,
    use_player: bool,
    window_width: u32,
    window_height: u32,
}

/// 解像度文字列を (width, height) にパースする
fn parse_resolution(s: &str) -> Result<(i32, i32), String> {
    match s.to_uppercase().as_str() {
        "QVGA" => Ok((320, 240)),
        "VGA" => Ok((640, 480)),
        "HD" => Ok((1280, 720)),
        "FHD" => Ok((1920, 1080)),
        "4K" => Ok((3840, 2160)),
        _ => {
            // WxH 形式
            let parts: Vec<&str> = s.split('x').collect();
            if parts.len() != 2 {
                return Err(format!(
                    "invalid resolution: '{}' (expected QVGA, VGA, HD, FHD, 4K, or WIDTHxHEIGHT)",
                    s
                ));
            }
            let w: i32 = parts[0]
                .parse()
                .map_err(|_| format!("invalid width in resolution: '{}'", parts[0]))?;
            let h: i32 = parts[1]
                .parse()
                .map_err(|_| format!("invalid height in resolution: '{}'", parts[1]))?;
            if w <= 0 || h <= 0 {
                return Err(format!("resolution must be positive: {}x{}", w, h));
            }
            Ok((w, h))
        }
    }
}

/// tracing_subscriber を初期化する
///
/// `--log-level` の値を tracing のフィルターに変換する。
/// verbose=TRACE, info=INFO, warning=WARN, error=ERROR, none=OFF。
/// 未指定時は INFO。
fn init_tracing(log_level: Option<&str>) {
    use tracing_subscriber::EnvFilter;

    let filter = match log_level {
        Some("verbose") => "trace",
        Some("info") | None => "info",
        Some("warning") => "warn",
        Some("error") => "error",
        Some("none") => "off",
        Some(other) => {
            eprintln!("unknown log level: '{}', using 'info'", other);
            "info"
        }
    };

    tracing_subscriber::fmt()
        .compact()
        .with_env_filter(EnvFilter::new(filter))
        .init();
}

// ─── サブコマンド用 RawArgs ────────────────────────────────────────────────

fn sub_args(args: &noargs::RawArgs) -> noargs::RawArgs {
    let help_mode = args.metadata().help_mode;
    let full_help = args.metadata().full_help;
    // RawArgs::new は最初の要素をプログラム名として消費するため、ダミーを先頭に追加する
    let remaining = args.remaining_args().map(|(_, s)| s.to_owned());
    let mut sub = noargs::RawArgs::new(std::iter::once(String::new()).chain(remaining));
    sub.metadata_mut().app_name = env!("CARGO_PKG_NAME");
    sub.metadata_mut().help_mode = help_mode;
    sub.metadata_mut().full_help = full_help;
    sub
}

// ─── P2P ───────────────────────────────────────────────────────────────────

async fn run_p2p(
    mut args: noargs::RawArgs,
    momo_config: MomoConfig,
    metrics_state: Option<std::sync::Arc<metrics::MetricsState>>,
) -> noargs::Result<()> {
    noargs::HELP_FLAG.take_help(&mut args);

    let port: u16 = noargs::opt("port")
        .ty("PORT")
        .doc("Port number")
        .default("8080")
        .take(&mut args)
        .then(|o| o.value().parse())?;

    let document_root: std::path::PathBuf = noargs::opt("document-root")
        .ty("PATH")
        .doc("HTTP document root directory")
        .default("html")
        .take(&mut args)
        .then(|o| -> Result<_, String> {
            let path: std::path::PathBuf = o
                .value()
                .parse()
                .map_err(|e: std::convert::Infallible| format!("{e}"))?;
            if !path.is_dir() {
                return Err(format!("directory does not exist: {}", path.display()));
            }
            Ok(path)
        })?;
    let video_codec_type: Option<String> = noargs::opt("video-codec-type")
        .ty("TYPE")
        .doc("Video codec type (VP8, VP9, AV1, H264, H265)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let audio_codec_type: Option<String> = noargs::opt("audio-codec-type")
        .ty("TYPE")
        .doc("Audio codec type (OPUS, PCMU, PCMA)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;

    if let Some(help) = args.finish()? {
        print!("{}", help);
        return Ok(());
    }

    // コーデックのバリデーション
    if let Some(ref codec) = video_codec_type {
        match codec.as_str() {
            "VP8" | "VP9" | "AV1" | "H264" | "H265" => {}
            other => {
                return Err(noargs::Error::other(
                    &noargs::raw_args(),
                    format!(
                        "不正な video-codec-type: {other} (VP8, VP9, AV1, H264, H265 のいずれかを指定してください)"
                    ),
                ));
            }
        }
    }
    if let Some(ref codec) = audio_codec_type {
        match codec.as_str() {
            "OPUS" | "PCMU" | "PCMA" => {}
            other => {
                return Err(noargs::Error::other(
                    &noargs::raw_args(),
                    format!(
                        "不正な audio-codec-type: {other} (OPUS, PCMU, PCMA のいずれかを指定してください)"
                    ),
                ));
            }
        }
    }

    let config = p2p::P2PConfig {
        port,
        document_root,
        no_google_stun: momo_config.no_google_stun,
        no_audio_device: momo_config.no_audio_device,
        no_video_input_device: momo_config.no_video_input_device,
        fake_capture_device: momo_config.fake_capture_device,
        use_libcamera: momo_config.use_libcamera,
        use_libcamera_native: momo_config.use_libcamera_native,
        libcamera_controls: momo_config.libcamera_controls,
        use_v4l2_encoder: momo_config.use_v4l2_encoder,
        openh264_lib: momo_config.openh264_lib,
        video_input_device: momo_config.video_input_device,
        audio_input_device: momo_config.audio_input_device,
        video_width: momo_config.video_width,
        video_height: momo_config.video_height,
        framerate: momo_config.framerate,
        force_pixel_format: momo_config
            .force_pixel_format
            .map(ForcePixelFormat::to_pixel_format),
        degradation_preference: momo_config.degradation_preference,
        video_codec_type,
        audio_codec_type,
        #[cfg(target_os = "linux")]
        serial: momo_config.serial,
        metrics_state,
    };

    p2p::run(config)
        .await
        .map_err(|e| noargs::Error::other(&noargs::raw_args(), e))
}

// ─── Ayame ─────────────────────────────────────────────────────────────────

#[cfg(feature = "ayame")]
async fn run_ayame(
    mut args: noargs::RawArgs,
    momo_config: MomoConfig,
    metrics_state: Option<std::sync::Arc<metrics::MetricsState>>,
) -> noargs::Result<()> {
    noargs::HELP_FLAG.take_help(&mut args);

    let signaling_url: String = noargs::opt("signaling-url")
        .ty("URL")
        .doc("Signaling URL")
        .example("wss://example.com/signaling")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let room_id: String = noargs::opt("room-id")
        .ty("ID")
        .doc("Room ID")
        .example("room-id")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let client_id: Option<String> = noargs::opt("client-id")
        .ty("ID")
        .doc("Client ID")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let signaling_key: Option<String> = noargs::opt("signaling-key")
        .ty("KEY")
        .doc("Signaling key")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let direction: String = noargs::opt("direction")
        .ty("DIRECTION")
        .doc("Direction (sendrecv, sendonly, recvonly)")
        .default("sendrecv")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let video_codec_type: Option<String> = noargs::opt("video-codec-type")
        .ty("TYPE")
        .doc("Video codec type (VP8, VP9, AV1, H264, H265)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let audio_codec_type: Option<String> = noargs::opt("audio-codec-type")
        .ty("TYPE")
        .doc("Audio codec type (OPUS, PCMU, PCMA)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;

    if let Some(help) = args.finish()? {
        print!("{}", help);
        return Ok(());
    }

    // コーデックのバリデーション
    if let Some(ref codec) = video_codec_type {
        match codec.as_str() {
            "VP8" | "VP9" | "AV1" | "H264" | "H265" => {}
            other => {
                return Err(noargs::Error::other(
                    &noargs::raw_args(),
                    format!(
                        "不正な video-codec-type: {other} (VP8, VP9, AV1, H264, H265 のいずれかを指定してください)"
                    ),
                ));
            }
        }
    }
    if let Some(ref codec) = audio_codec_type {
        match codec.as_str() {
            "OPUS" | "PCMU" | "PCMA" => {}
            other => {
                return Err(noargs::Error::other(
                    &noargs::raw_args(),
                    format!(
                        "不正な audio-codec-type: {other} (OPUS, PCMU, PCMA のいずれかを指定してください)"
                    ),
                ));
            }
        }
    }

    let direction = match direction.as_str() {
        "sendrecv" => ayame::Direction::SendRecv,
        "sendonly" => ayame::Direction::SendOnly,
        "recvonly" => ayame::Direction::RecvOnly,
        other => {
            return Err(noargs::Error::other(
                &noargs::raw_args(),
                format!("不正な direction: {other}"),
            ));
        }
    };

    let config = ayame::AyameConfig {
        signaling_url,
        room_id,
        client_id,
        signaling_key,
        direction,
        no_google_stun: momo_config.no_google_stun,
        no_audio_device: momo_config.no_audio_device,
        no_video_input_device: momo_config.no_video_input_device,
        fake_capture_device: momo_config.fake_capture_device,
        use_libcamera: momo_config.use_libcamera,
        use_libcamera_native: momo_config.use_libcamera_native,
        libcamera_controls: momo_config.libcamera_controls,
        use_v4l2_encoder: momo_config.use_v4l2_encoder,
        openh264_lib: momo_config.openh264_lib,
        video_input_device: momo_config.video_input_device,
        audio_input_device: momo_config.audio_input_device,
        video_width: momo_config.video_width,
        video_height: momo_config.video_height,
        framerate: momo_config.framerate,
        insecure: momo_config.insecure,
        force_pixel_format: momo_config
            .force_pixel_format
            .map(ForcePixelFormat::to_pixel_format),
        client_cert: momo_config.client_cert.clone(),
        ca_cert: momo_config.ca_cert.clone(),
        degradation_preference: momo_config.degradation_preference,
        video_codec_type,
        audio_codec_type,
        #[cfg(target_os = "linux")]
        serial: momo_config.serial,
    };

    ayame::run(config, metrics_state)
        .await
        .map_err(|e| noargs::Error::other(&noargs::raw_args(), e))
}

// ─── ビデオコーデックエンジン一覧 ────────────────────────────────────────────

fn print_video_codec_engines() {
    use shiguredo_webrtc::VideoCodecType;

    let codec_types = [
        VideoCodecType::Vp8,
        VideoCodecType::Vp9,
        VideoCodecType::Av1,
        VideoCodecType::H264,
        VideoCodecType::H265,
    ];

    #[cfg(feature = "sora")]
    {
        use sora_sdk::{CodecDirection, InternalVideoCodecCapability, VideoCodecCapability};

        let mut capabilities: Vec<Box<dyn VideoCodecCapability>> = vec![];

        // ソフトウェアコーデック (全プラットフォーム)
        capabilities.push(Box::new(InternalVideoCodecCapability::new()));

        // Apple VideoToolbox (macOS/iOS)
        #[cfg(any(target_os = "macos", target_os = "ios"))]
        {
            use sora_sdk::InternalAppleVideoCodecCapability;
            if let Some(hwa) = InternalAppleVideoCodecCapability::new() {
                capabilities.push(Box::new(hwa));
            }
        }

        for capability in &capabilities {
            let impl_info = capability.get_implementation();
            println!("{}:", impl_info.name());
            println!("  {}", impl_info.description());
            for &codec_type in &codec_types {
                let codec_name = codec_type.as_str().unwrap_or("Unknown");
                let enc = capability.is_supported(CodecDirection::Encoder, codec_type);
                let dec = capability.is_supported(CodecDirection::Decoder, codec_type);
                if enc || dec {
                    let mut parts = Vec::new();
                    if enc {
                        parts.push("Encoder");
                    }
                    if dec {
                        parts.push("Decoder");
                    }
                    println!("  {codec_name}: {}", parts.join(", "));
                }
            }
            println!();
        }
    }

    #[cfg(not(feature = "sora"))]
    {
        // sora feature が無効の場合はソフトウェアコーデックの情報のみ表示する
        println!("internal:");
        println!("  WebRTC built-in VideoCodecFactory");
        for &codec_type in &codec_types {
            if let Some(name) = codec_type.as_str() {
                println!("  {name}: Encoder, Decoder");
            }
        }
        println!();
    }
}

// ─── デバイス一覧 ───────────────────────────────────────────────────────────

fn print_devices_json() -> Result<(), crate::error::BoxError> {
    use shiguredo_audio_device::AudioDeviceList;
    use shiguredo_video_device::{PixelFormat, VideoDeviceList};

    struct VFormat {
        width: i32,
        height: i32,
        min_fps: f32,
        max_fps: f32,
        pixel_format: String,
    }
    struct VDevice {
        name: String,
        unique_id: String,
        formats: Vec<VFormat>,
    }
    struct ADevice {
        name: String,
        unique_id: String,
        channels: i32,
        sample_rate: i32,
    }

    let video_list =
        VideoDeviceList::enumerate().map_err(|e| format!("映像デバイスの列挙に失敗: {e}"))?;
    let audio_list = AudioDeviceList::enumerate_input()
        .map_err(|e| format!("音声入力デバイスの列挙に失敗: {e}"))?;

    let videos: Vec<VDevice> = (&video_list)
        .into_iter()
        .map(|d| VDevice {
            name: d.name().unwrap_or_default(),
            unique_id: d.unique_id().unwrap_or_default(),
            formats: d
                .formats()
                .into_iter()
                .map(|f| VFormat {
                    width: f.width,
                    height: f.height,
                    min_fps: f.min_fps,
                    max_fps: f.max_fps,
                    pixel_format: match f.pixel_format {
                        PixelFormat::Nv12 => "nv12".to_string(),
                        PixelFormat::Yuy2 => "yuy2".to_string(),
                        PixelFormat::I420 => "i420".to_string(),
                        PixelFormat::Mjpeg => "mjpeg".to_string(),
                        PixelFormat::Unknown(n) => format!("unknown({n})"),
                    },
                })
                .collect(),
        })
        .collect();

    let audios: Vec<ADevice> = audio_list
        .devices()
        .iter()
        .map(|d| ADevice {
            name: d.name().unwrap_or_default(),
            unique_id: d.unique_id().unwrap_or_default(),
            channels: d.channels(),
            sample_rate: d.sample_rate(),
        })
        .collect();

    let json = nojson::object(|f| {
        f.member(
            "video",
            nojson::array(|f| {
                for v in &videos {
                    f.element(nojson::object(|f| {
                        f.member("name", v.name.as_str())?;
                        f.member("unique_id", v.unique_id.as_str())?;
                        f.member(
                            "formats",
                            nojson::array(|f| {
                                for fmt in &v.formats {
                                    f.element(nojson::object(|f| {
                                        f.member("width", fmt.width)?;
                                        f.member("height", fmt.height)?;
                                        f.member("min_fps", fmt.min_fps)?;
                                        f.member("max_fps", fmt.max_fps)?;
                                        f.member("pixel_format", fmt.pixel_format.as_str())
                                    }))?;
                                }
                                Ok(())
                            }),
                        )
                    }))?;
                }
                Ok(())
            }),
        )?;
        f.member(
            "audio_input",
            nojson::array(|f| {
                for a in &audios {
                    f.element(nojson::object(|f| {
                        f.member("name", a.name.as_str())?;
                        f.member("unique_id", a.unique_id.as_str())?;
                        f.member("channels", a.channels)?;
                        f.member("sample_rate", a.sample_rate)
                    }))?;
                }
                Ok(())
            }),
        )
    });

    println!("{json}");
    Ok(())
}

// ─── Sora ──────────────────────────────────────────────────────────────────

async fn run_sora(
    mut args: noargs::RawArgs,
    momo_config: MomoConfig,
    metrics_state: Option<std::sync::Arc<metrics::MetricsState>>,
) -> noargs::Result<()> {
    noargs::HELP_FLAG.take_help(&mut args);

    let signaling_urls: String = noargs::opt("signaling-urls")
        .ty("URLS")
        .doc("Signaling URLs")
        .example("wss://example.com/signaling")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let channel_id: String = noargs::opt("channel-id")
        .ty("ID")
        .doc("Channel ID")
        .example("channel-id")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let _auto = noargs::flag("auto")
        .doc("Connect to Sora automatically")
        .take(&mut args)
        .is_present();
    let video: String = noargs::opt("video")
        .ty("BOOL")
        .doc("Send video to sora (default: true)")
        .default("true")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let audio: String = noargs::opt("audio")
        .ty("BOOL")
        .doc("Send audio to sora (default: true)")
        .default("true")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let video_codec_type: Option<String> = noargs::opt("video-codec-type")
        .ty("TYPE")
        .doc("Video codec for send")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let audio_codec_type: Option<String> = noargs::opt("audio-codec-type")
        .ty("TYPE")
        .doc("Audio codec for send")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let video_bit_rate: u32 = noargs::opt("video-bit-rate")
        .ty("RATE")
        .doc("Video bit rate")
        .default("0")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let audio_bit_rate: u32 = noargs::opt("audio-bit-rate")
        .ty("RATE")
        .doc("Audio bit rate")
        .default("0")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let role: String = noargs::opt("role")
        .ty("ROLE")
        .doc("Role (default: sendonly)")
        .default("sendonly")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let spotlight: String = noargs::opt("spotlight")
        .ty("BOOL")
        .doc("Use spotlight")
        .default("false")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let _port: i32 = noargs::opt("port")
        .ty("PORT")
        .doc("Port number (default: -1)")
        .default("-1")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let simulcast: String = noargs::opt("simulcast")
        .ty("BOOL")
        .doc("Use simulcast (default: false)")
        .default("false")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let data_channel_signaling: Option<String> = noargs::opt("data-channel-signaling")
        .ty("BOOL")
        .doc("Use DataChannel for Sora signaling (default: none)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let _data_channel_signaling_timeout: u32 = noargs::opt("data-channel-signaling-timeout")
        .ty("SECONDS")
        .doc("Timeout for Data Channel in seconds (default: 180)")
        .default("180")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let ignore_disconnect_websocket: Option<String> = noargs::opt("ignore-disconnect-websocket")
        .ty("BOOL")
        .doc("Ignore WebSocket disconnection if using Data Channel (default: none)")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;
    let disconnect_wait_timeout: u32 = noargs::opt("disconnect-wait-timeout")
        .ty("SECONDS")
        .doc("Disconnecting timeout for Data Channel in seconds (default: 5)")
        .default("5")
        .take(&mut args)
        .then(|o| o.value().parse())?;
    let metadata: Option<String> = noargs::opt("metadata")
        .ty("JSON")
        .doc("Signaling metadata used in connect message")
        .take(&mut args)
        .present_and_then(|o| o.value().parse())?;

    if let Some(help) = args.finish()? {
        print!("{}", help);
        return Ok(());
    }

    #[cfg(not(feature = "sora"))]
    {
        // 未使用変数を抑制
        let _ = (
            &signaling_urls,
            &channel_id,
            &video,
            &audio,
            &video_codec_type,
            &audio_codec_type,
            &video_bit_rate,
            &audio_bit_rate,
            &role,
            &spotlight,
            &simulcast,
            &data_channel_signaling,
            &ignore_disconnect_websocket,
            &disconnect_wait_timeout,
            &metadata,
            &momo_config,
            &metrics_state,
        );
        Err(noargs::Error::other(
            &noargs::raw_args(),
            "sora mode requires the 'sora' feature to be enabled",
        ))
    }

    #[cfg(feature = "sora")]
    {
        // bool 文字列のパース
        let parse_bool = |s: &str, name: &str| -> noargs::Result<bool> {
            match s {
                "true" => Ok(true),
                "false" => Ok(false),
                other => Err(noargs::Error::other(
                    &noargs::raw_args(),
                    format!("不正な {name}: {other} (true または false を指定してください)"),
                )),
            }
        };
        let video = parse_bool(&video, "video")?;
        let audio = parse_bool(&audio, "audio")?;
        let spotlight = parse_bool(&spotlight, "spotlight")?;
        let simulcast = parse_bool(&simulcast, "simulcast")?;

        // role のバリデーション
        let role = sora_sdk::Role::parse(&role)
            .map_err(|e| noargs::Error::other(&noargs::raw_args(), format!("不正な role: {e}")))?;

        // video-codec-type のバリデーション
        if let Some(ref codec) = video_codec_type {
            match codec.as_str() {
                "VP8" | "VP9" | "AV1" | "H264" | "H265" => {}
                other => {
                    return Err(noargs::Error::other(
                        &noargs::raw_args(),
                        format!(
                            "不正な video-codec-type: {other} (VP8, VP9, AV1, H264, H265 のいずれかを指定してください)"
                        ),
                    ));
                }
            }
        }

        // audio-codec-type のバリデーション
        if let Some(ref codec) = audio_codec_type {
            match codec.as_str() {
                "OPUS" => {}
                other => {
                    return Err(noargs::Error::other(
                        &noargs::raw_args(),
                        format!("不正な audio-codec-type: {other} (OPUS のみ対応しています)"),
                    ));
                }
            }
        }

        // data-channel-signaling / ignore-disconnect-websocket の bool パース
        let data_channel_signaling = data_channel_signaling
            .as_deref()
            .map(|s| parse_bool(s, "data-channel-signaling"))
            .transpose()?;
        let ignore_disconnect_websocket = ignore_disconnect_websocket
            .as_deref()
            .map(|s| parse_bool(s, "ignore-disconnect-websocket"))
            .transpose()?;

        // signaling-urls をカンマ区切りで分割
        let signaling_urls: Vec<String> = signaling_urls
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();

        #[allow(unused_mut)]
        let mut config = sora::SoraConfig {
            signaling_urls,
            channel_id,
            role,
            video,
            audio,
            video_codec_type,
            audio_codec_type,
            video_bit_rate,
            audio_bit_rate,
            h264_encoder: momo_config.h264_encoder,
            h264_decoder: momo_config.h264_decoder,
            h265_encoder: momo_config.h265_encoder,
            h265_decoder: momo_config.h265_decoder,
            spotlight,
            simulcast,
            data_channel_signaling,
            ignore_disconnect_websocket,
            metadata,
            no_audio_device: momo_config.no_audio_device,
            no_video_input_device: momo_config.no_video_input_device,
            fake_capture_device: momo_config.fake_capture_device,
            use_v4l2_encoder: momo_config.use_v4l2_encoder,
            openh264_lib: momo_config.openh264_lib,
            use_libcamera: momo_config.use_libcamera,
            use_libcamera_native: momo_config.use_libcamera_native,
            libcamera_controls: momo_config.libcamera_controls,
            video_input_device: momo_config.video_input_device,
            audio_input_device: momo_config.audio_input_device,
            video_width: momo_config.video_width,
            video_height: momo_config.video_height,
            framerate: momo_config.framerate,
            disconnect_wait_timeout,
            insecure: momo_config.insecure,
            force_pixel_format: momo_config
                .force_pixel_format
                .map(ForcePixelFormat::to_pixel_format),
            client_cert: momo_config.client_cert,
            ca_cert: momo_config.ca_cert,
            proxy_url: momo_config.proxy_url,
            proxy_username: momo_config.proxy_username,
            proxy_password: momo_config.proxy_password,
            beep_trigger: None,
            #[cfg(feature = "player")]
            preview_tx: None,
        };

        // fake capture 時は FakeAudioCapturer を起動してビープ音を生成する
        let mut _fake_audio_capturer = None;
        let mut fake_adm = None;
        if momo_config.fake_capture_device && !momo_config.no_audio_device {
            let trigger = crate::fake::BeepTrigger::new();
            let mut capturer = crate::fake::FakeAudioCapturer::new(trigger.clone());
            capturer.start();
            config.beep_trigger = Some(trigger);
            fake_adm = Some(capturer.audio_device_module());
            _fake_audio_capturer = Some(capturer);
        }

        // player feature なしで --player を指定した場合の警告
        #[cfg(not(feature = "player"))]
        if momo_config.use_player {
            return Err(noargs::Error::other(
                &noargs::raw_args(),
                "--player requires the 'player' feature to be enabled",
            ));
        }

        // プレビュー有効時: Sora 接続を別タスクで起動し、メインスレッドで SDL3 ループを回す
        #[cfg(feature = "player")]
        if momo_config.use_player && role.wants_send() {
            let (preview_tx, preview_rx) = preview::create_preview_channel();
            let (shutdown_tx, shutdown_rx) = std::sync::mpsc::sync_channel::<()>(1);
            config.preview_tx = Some(preview_tx);

            let window_width = momo_config.window_width as i32;
            let window_height = momo_config.window_height as i32;

            let fake_adm_for_spawn = fake_adm.take().map(crate::fake::SendableAdm);
            let sora_handle = tokio::spawn(async move {
                let result = sora::run(config, metrics_state, fake_adm_for_spawn).await;
                // Sora 接続が終了したらプレビューループに通知する
                let _ = shutdown_tx.try_send(());
                result
            });

            // メインスレッドで SDL3 イベントループを実行する
            tokio::task::block_in_place(|| {
                if let Err(e) =
                    preview::run_preview_loop(window_width, window_height, preview_rx, shutdown_rx)
                {
                    tracing::error!(target: "preview", error = %e, "preview loop error");
                }
            });

            return sora_handle
                .await
                .map_err(|e| noargs::Error::other(&noargs::raw_args(), format!("{e}")))?
                .map_err(|e| noargs::Error::other(&noargs::raw_args(), e));
        }

        sora::run(
            config,
            metrics_state,
            fake_adm.map(crate::fake::SendableAdm),
        )
        .await
        .map_err(|e| noargs::Error::other(&noargs::raw_args(), e))
    }
}
