/// register に対する accept メッセージを生成する
pub(super) fn json_accept(is_exist_user: bool) -> String {
    nojson::object(|f| {
        f.member("type", "accept")?;
        f.member("isExistUser", is_exist_user)
    })
    .to_string()
}

/// answer メッセージを生成する
pub(super) fn json_answer(sdp: &str) -> String {
    nojson::object(|f| {
        f.member("type", "answer")?;
        f.member("sdp", sdp)
    })
    .to_string()
}

/// ICE candidate メッセージを生成する
pub(super) fn json_candidate(sdp: &str, sdp_mid: &str, sdp_mline_index: i32) -> String {
    nojson::object(|f| {
        f.member("type", "candidate")?;
        f.member(
            "ice",
            nojson::object(|f| {
                f.member("candidate", sdp)?;
                f.member("sdpMid", sdp_mid)?;
                f.member("sdpMLineIndex", sdp_mline_index)
            }),
        )
    })
    .to_string()
}
