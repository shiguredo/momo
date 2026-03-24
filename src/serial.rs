//! シリアルポート ↔ DataChannel 双方向ブリッジ
//!
//! `--serial DEVICE,BAUDRATE` で指定されたシリアルポートを開き、
//! ブラウザが作成した "serial" ラベルの DataChannel と双方向にデータを中継する。

use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::unix::io::AsRawFd;
use std::str::FromStr;

use shiguredo_webrtc::{DataChannel, DataChannelObserver, DataChannelObserverHandler};
use tokio::io::unix::AsyncFd;
use tokio::sync::mpsc;
use tracing::{error, info, warn};

/// シリアルポート設定
#[derive(Clone, Debug)]
pub struct SerialConfig {
    pub device: String,
    pub baudrate: u32,
}

impl FromStr for SerialConfig {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let (device, baudrate_str) = s
            .rsplit_once(',')
            .ok_or_else(|| format!("invalid serial format: '{s}' (expected DEVICE,BAUDRATE)"))?;
        if device.is_empty() {
            return Err("serial device path is empty".to_string());
        }
        let baudrate: u32 = baudrate_str
            .parse()
            .map_err(|_| format!("invalid baudrate: '{baudrate_str}'"))?;
        // ボーレートの妥当性は open_serial で検証する
        Ok(SerialConfig {
            device: device.to_string(),
            baudrate,
        })
    }
}

/// ボーレート値を libc 定数に変換する
fn to_speed(baudrate: u32) -> Result<libc::speed_t, String> {
    match baudrate {
        1200 => Ok(libc::B1200),
        2400 => Ok(libc::B2400),
        4800 => Ok(libc::B4800),
        9600 => Ok(libc::B9600),
        19200 => Ok(libc::B19200),
        38400 => Ok(libc::B38400),
        57600 => Ok(libc::B57600),
        115200 => Ok(libc::B115200),
        230400 => Ok(libc::B230400),
        460800 => Ok(libc::B460800),
        500000 => Ok(libc::B500000),
        576000 => Ok(libc::B576000),
        921600 => Ok(libc::B921600),
        1000000 => Ok(libc::B1000000),
        1500000 => Ok(libc::B1500000),
        2000000 => Ok(libc::B2000000),
        _ => Err(format!("unsupported baudrate: {baudrate}")),
    }
}

/// シリアルポートを開いて非同期 fd として返す
fn open_serial(config: &SerialConfig) -> Result<AsyncFd<File>, String> {
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .open(&config.device)
        .map_err(|e| format!("failed to open {}: {e}", config.device))?;

    let fd = file.as_raw_fd();
    let speed = to_speed(config.baudrate)?;

    // termios で raw モード + ボーレート設定
    unsafe {
        let mut termios: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut termios) != 0 {
            return Err(format!("tcgetattr failed: {}", io::Error::last_os_error()));
        }

        libc::cfmakeraw(&mut termios);
        if libc::cfsetispeed(&mut termios, speed) != 0 {
            return Err(format!(
                "cfsetispeed failed: {}",
                io::Error::last_os_error()
            ));
        }
        if libc::cfsetospeed(&mut termios, speed) != 0 {
            return Err(format!(
                "cfsetospeed failed: {}",
                io::Error::last_os_error()
            ));
        }

        // CLOCAL: モデム制御を無効化、CREAD: 受信を有効化
        termios.c_cflag |= libc::CLOCAL | libc::CREAD;

        // VMIN=1, VTIME=0: 最低 1 バイト受信するまでブロック (non-blocking fd なので実質即時)
        termios.c_cc[libc::VMIN] = 1;
        termios.c_cc[libc::VTIME] = 0;

        if libc::tcsetattr(fd, libc::TCSANOW, &termios) != 0 {
            return Err(format!("tcsetattr failed: {}", io::Error::last_os_error()));
        }

        // non-blocking モードに設定
        let flags = libc::fcntl(fd, libc::F_GETFL);
        if flags < 0 {
            return Err(format!(
                "fcntl F_GETFL failed: {}",
                io::Error::last_os_error()
            ));
        }
        if libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) < 0 {
            return Err(format!(
                "fcntl F_SETFL failed: {}",
                io::Error::last_os_error()
            ));
        }
    }

    AsyncFd::new(file).map_err(|e| format!("AsyncFd::new failed: {e}"))
}

/// DataChannel の on_message で受信したデータをチャンネルに転送するハンドラ
struct DcHandler {
    tx: mpsc::UnboundedSender<Vec<u8>>,
}

impl DataChannelObserverHandler for DcHandler {
    fn on_state_change(&mut self) {}

    fn on_message(&mut self, data: &[u8], _is_binary: bool) {
        let _ = self.tx.send(data.to_vec());
    }
}

/// シリアルポートと DataChannel を双方向に接続するブリッジを起動する
///
/// DataChannel が "serial" ラベルで Open になった時点で呼び出す。
/// DataChannel が閉じるか、シリアルポートでエラーが発生したら終了する。
pub fn start_serial_bridge(config: &SerialConfig, mut dc: DataChannel) {
    let serial_fd = match open_serial(config) {
        Ok(fd) => fd,
        Err(e) => {
            error!(target: "serial", error = %e, "failed to open serial port");
            return;
        }
    };

    info!(
        target: "serial",
        device = %config.device,
        baudrate = config.baudrate,
        "serial bridge started"
    );

    // DataChannel → シリアルポートのチャンネル
    let (dc_data_tx, mut dc_data_rx) = mpsc::unbounded_channel::<Vec<u8>>();

    let dc_observer = DataChannelObserver::new_with_handler(Box::new(DcHandler { tx: dc_data_tx }));
    dc.register_observer(&dc_observer);

    // tokio タスクでブリッジを実行
    // on_data_channel コールバックは C++ スレッドから呼ばれるため、
    // tokio::runtime::Handle で現在のランタイムにタスクをスポーンする
    let handle = tokio::runtime::Handle::current();
    handle.spawn(async move {
        // _dc_observer をタスク内に移動して DataChannel より後に drop させる
        let _dc_observer = dc_observer;
        let mut read_buf = vec![0u8; 4096];

        loop {
            tokio::select! {
                // シリアルポート → DataChannel
                readable = serial_fd.readable() => {
                    match readable {
                        Ok(guard) => {
                            match guard.try_io(|inner| {
                                let mut file_ref = inner.get_ref();
                                file_ref.read(&mut read_buf)
                            }) {
                                Ok(Ok(0)) => {
                                    info!(target: "serial", "serial port EOF");
                                    break;
                                }
                                Ok(Ok(n)) => {
                                    if !dc.send(&read_buf[..n], true) {
                                        warn!(target: "serial", "DataChannel send failed");
                                        break;
                                    }
                                }
                                Ok(Err(e)) => {
                                    error!(target: "serial", error = %e, "serial read error");
                                    break;
                                }
                                Err(_would_block) => {
                                    // try_io が readiness をクリア済み
                                }
                            }
                        }
                        Err(e) => {
                            error!(target: "serial", error = %e, "serial readable error");
                            break;
                        }
                    }
                }

                // DataChannel → シリアルポート
                data = dc_data_rx.recv() => {
                    match data {
                        Some(data) => {
                            match write_all_async(&serial_fd, &data).await {
                                Ok(()) => {}
                                Err(e) => {
                                    error!(target: "serial", error = %e, "serial write error");
                                    break;
                                }
                            }
                        }
                        None => {
                            // DataChannel Observer が drop された
                            info!(target: "serial", "DataChannel closed");
                            break;
                        }
                    }
                }
            }
        }

        info!(target: "serial", "serial bridge stopped");
        dc.close();
    });
}

/// AsyncFd を使って全データを非同期に書き込む
async fn write_all_async(fd: &AsyncFd<File>, mut data: &[u8]) -> io::Result<()> {
    while !data.is_empty() {
        let guard = fd.writable().await?;
        match guard.try_io(|inner| {
            let mut file_ref = inner.get_ref();
            file_ref.write(data)
        }) {
            Ok(Ok(n)) => {
                data = &data[n..];
            }
            Ok(Err(e)) => return Err(e),
            Err(_would_block) => {
                // try_io が readiness をクリア済み
            }
        }
    }
    Ok(())
}
