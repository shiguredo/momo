use nojson::{DisplayJson, JsonFormatter, RawJson};

use crate::error::BoxError;

// ─── ICE サーバー ─────────────────────────────────────────────────────────────

pub(super) struct IceServerConfig {
    pub(super) urls: Vec<String>,
    pub(super) username: Option<String>,
    pub(super) credential: Option<String>,
}

// ─── 受信メッセージ型 ─────────────────────────────────────────────────────────

pub(super) enum ReceivedMessage {
    Accept(AcceptMessage),
    Reject {
        reason: String,
    },
    Offer {
        sdp: String,
    },
    Answer {
        sdp: String,
    },
    Candidate {
        candidate: String,
        sdp_mid: String,
        sdp_mline_index: i32,
    },
    Ping,
    Bye,
}

pub(super) struct AcceptMessage {
    pub(super) is_exist_user: bool,
    pub(super) ice_servers: Vec<IceServerConfig>,
}

// ─── 送信メッセージ (DisplayJson) ─────────────────────────────────────────────

pub(super) struct RegisterMessage<'a> {
    pub(super) room_id: &'a str,
    pub(super) client_id: &'a str,
    pub(super) signaling_key: Option<&'a str>,
}

impl DisplayJson for RegisterMessage<'_> {
    fn fmt(&self, f: &mut JsonFormatter<'_, '_>) -> std::fmt::Result {
        f.object(|f| {
            f.member("type", "register")?;
            f.member("roomId", self.room_id)?;
            f.member("clientId", self.client_id)?;
            if let Some(key) = self.signaling_key {
                f.member("key", key)?;
            }
            Ok(())
        })
    }
}

pub(super) struct SdpMessage<'a> {
    pub(super) sdp_type: &'a str,
    pub(super) sdp: &'a str,
}

impl DisplayJson for SdpMessage<'_> {
    fn fmt(&self, f: &mut JsonFormatter<'_, '_>) -> std::fmt::Result {
        f.object(|f| {
            f.member("type", self.sdp_type)?;
            f.member("sdp", self.sdp)
        })
    }
}

pub(super) struct CandidateMessage<'a> {
    pub(super) candidate: &'a str,
    pub(super) sdp_mid: &'a str,
    pub(super) sdp_mline_index: i32,
}

impl DisplayJson for CandidateMessage<'_> {
    fn fmt(&self, f: &mut JsonFormatter<'_, '_>) -> std::fmt::Result {
        f.object(|f| {
            f.member("type", "candidate")?;
            f.member(
                "ice",
                IceField {
                    candidate: self.candidate,
                    sdp_mid: self.sdp_mid,
                    sdp_mline_index: self.sdp_mline_index,
                },
            )
        })
    }
}

struct IceField<'a> {
    candidate: &'a str,
    sdp_mid: &'a str,
    sdp_mline_index: i32,
}

impl DisplayJson for IceField<'_> {
    fn fmt(&self, f: &mut JsonFormatter<'_, '_>) -> std::fmt::Result {
        f.object(|f| {
            f.member("candidate", self.candidate)?;
            f.member("sdpMid", self.sdp_mid)?;
            f.member("sdpMLineIndex", self.sdp_mline_index)
        })
    }
}

pub(super) struct PongMessage;

impl DisplayJson for PongMessage {
    fn fmt(&self, f: &mut JsonFormatter<'_, '_>) -> std::fmt::Result {
        f.object(|f| f.member("type", "pong"))
    }
}

// ─── JSON パース ──────────────────────────────────────────────────────────────

pub(super) fn parse_received_message(text: &str) -> Result<ReceivedMessage, BoxError> {
    let json = RawJson::parse(text).map_err(|e| format!("JSON パースエラー: {e}"))?;
    let value = json.value();

    let msg_type: String = value
        .to_member("type")?
        .required()?
        .try_into()
        .map_err(|e: nojson::JsonParseError| format!("type フィールドエラー: {e}"))?;

    match msg_type.as_str() {
        "accept" => {
            let is_exist_user: Option<bool> = value
                .to_member("isExistUser")?
                .try_into()
                .map_err(|e: nojson::JsonParseError| format!("isExistUser エラー: {e}"))?;
            let ice_servers = parse_ice_servers(&value)?;
            Ok(ReceivedMessage::Accept(AcceptMessage {
                is_exist_user: is_exist_user.unwrap_or(false),
                ice_servers,
            }))
        }
        "reject" => {
            let reason: Option<String> = value
                .to_member("reason")?
                .try_into()
                .map_err(|e: nojson::JsonParseError| format!("reason エラー: {e}"))?;
            Ok(ReceivedMessage::Reject {
                reason: reason.unwrap_or_default(),
            })
        }
        "offer" => {
            let sdp: String = value
                .to_member("sdp")?
                .required()?
                .try_into()
                .map_err(|e: nojson::JsonParseError| format!("sdp エラー: {e}"))?;
            Ok(ReceivedMessage::Offer { sdp })
        }
        "answer" => {
            let sdp: String = value
                .to_member("sdp")?
                .required()?
                .try_into()
                .map_err(|e: nojson::JsonParseError| format!("sdp エラー: {e}"))?;
            Ok(ReceivedMessage::Answer { sdp })
        }
        "candidate" => {
            let ice = value.to_member("ice")?.required()?;
            let candidate: String = ice
                .to_member("candidate")?
                .required()?
                .try_into()
                .map_err(|e: nojson::JsonParseError| format!("candidate エラー: {e}"))?;
            let sdp_mid: Option<String> = ice
                .to_member("sdpMid")?
                .try_into()
                .map_err(|e: nojson::JsonParseError| format!("sdpMid エラー: {e}"))?;
            let sdp_mline_index: Option<i32> = ice
                .to_member("sdpMLineIndex")?
                .try_into()
                .map_err(|e: nojson::JsonParseError| format!("sdpMLineIndex エラー: {e}"))?;
            Ok(ReceivedMessage::Candidate {
                candidate,
                sdp_mid: sdp_mid.unwrap_or_default(),
                sdp_mline_index: sdp_mline_index.unwrap_or(0),
            })
        }
        "ping" => Ok(ReceivedMessage::Ping),
        "bye" => Ok(ReceivedMessage::Bye),
        other => Err(format!("不明なメッセージタイプ: {other}").into()),
    }
}

fn parse_ice_servers(
    value: &nojson::RawJsonValue<'_, '_>,
) -> Result<Vec<IceServerConfig>, BoxError> {
    let ice_servers_opt: Option<nojson::RawJsonValue<'_, '_>> =
        value.to_member("iceServers")?.map(Ok)?;
    let Some(ice_servers) = ice_servers_opt else {
        return Ok(Vec::new());
    };

    let arr = ice_servers
        .to_array()
        .map_err(|e| format!("iceServers 配列エラー: {e}"))?;

    let mut servers = Vec::new();
    for item in arr {
        let urls_value = item.to_member("urls")?.required()?;
        let urls_arr = urls_value
            .to_array()
            .map_err(|e| format!("urls 配列エラー: {e}"))?;
        let mut urls = Vec::new();
        for url in urls_arr {
            let s: String = url
                .try_into()
                .map_err(|e: nojson::JsonParseError| format!("URL エラー: {e}"))?;
            urls.push(s);
        }
        let username: Option<String> = item
            .to_member("username")?
            .try_into()
            .map_err(|e: nojson::JsonParseError| format!("username エラー: {e}"))?;
        let credential: Option<String> = item
            .to_member("credential")?
            .try_into()
            .map_err(|e: nojson::JsonParseError| format!("credential エラー: {e}"))?;
        servers.push(IceServerConfig {
            urls,
            username,
            credential,
        });
    }

    Ok(servers)
}
