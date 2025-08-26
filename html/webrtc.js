(() => {
  'use strict';

  // DOM 参照
  const remoteVideo = document.getElementById('remote_video');
  const dataTextInput = document.getElementById('data_text');
  remoteVideo.controls = true;

  // 状態
  let peerConnection = null;
  let dataChannel = null;
  let candidates = [];
  let hasReceivedSdp = false;

  // ICE サーバー設定
  const iceServers = [{ urls: 'stun:stun.l.google.com:19302' }];
  const peerConnectionConfig = { iceServers };

  // WebSocket 設定
  const isSSL = location.protocol === 'https:';
  const wsProtocol = isSSL ? 'wss://' : 'ws://';
  const wsUrl = wsProtocol + location.host + '/ws';
  const ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    console.log('ws open()');
  };
  ws.onerror = (error) => {
    console.error('ws onerror() ERROR:', error);
  };
  ws.onmessage = (event) => {
    console.log('ws onmessage() data:', event.data);
    const message = JSON.parse(event.data);
    switch (message.type) {
      case 'offer': {
        console.log('Received offer ...');
        const offer = new RTCSessionDescription(message);
        console.log('offer: ', offer);
        setOffer(offer);
        break;
      }
      case 'answer': {
        console.log('Received answer ...');
        const answer = new RTCSessionDescription(message);
        console.log('answer: ', answer);
        setAnswer(answer);
        break;
      }
      case 'candidate': {
        console.log('Received ICE candidate ...');
        const candidate = new RTCIceCandidate(message.ice);
        console.log('candidate: ', candidate);
        if (hasReceivedSdp) {
          addIceCandidate(candidate);
        } else {
          candidates.push(candidate);
        }
        break;
      }
      case 'close': {
        console.log('peer connection is closed ...');
        break;
      }
      default:
        console.warn('Unknown WS message type:', message.type);
    }
  };

  // ページ読み込み時に能力からコーデック一覧を構築
  try { populateVideoCodecSelect(); } catch (e) { console.warn('populateVideoCodecSelect failed:', e); }
  try { bindUi(); } catch (e) { console.warn('bindUi failed:', e); }

  // 公開 API
  function connect() {
    console.group();
    if (!peerConnection) {
      console.log('make Offer');
      makeOffer();
    } else {
      console.warn('peer connection already exists.');
    }
    console.groupEnd();
  }

  function disconnect() {
    console.group();
    if (peerConnection && peerConnection.iceConnectionState !== 'closed') {
      try {
        peerConnection.close();
      } catch (e) {
        console.warn('close() error:', e);
      }
      peerConnection = null;
      if (dataChannel) {
        try { dataChannel.close(); } catch (_) {}
        dataChannel = null;
      }
      // 相手側へ通知
      if (ws && ws.readyState === 1) {
        ws.send(JSON.stringify({ type: 'close' }));
        console.log('sending close message');
      }
      cleanupVideoElement(remoteVideo);
      candidates = [];
      hasReceivedSdp = false;
      try { updateUiState(); } catch (_) {}
    } else {
      console.log('peerConnection is closed.');
    }
    console.groupEnd();
  }

  function play() {
    remoteVideo.play();
  }

  function sendDataChannel() {
    const textData = dataTextInput.value;
    if (textData.length === 0) return;
    if (!dataChannel || dataChannel.readyState !== 'open') return;
    dataChannel.send(new TextEncoder().encode(textData));
    dataTextInput.value = '';
  }

  // 内部処理
  function drainCandidate() {
    hasReceivedSdp = true;
    candidates.forEach((candidate) => addIceCandidate(candidate));
    candidates = [];
  }

  function addIceCandidate(candidate) {
    if (!peerConnection) {
      console.error('PeerConnection does not exist!');
      return;
    }
    try {
      peerConnection.addIceCandidate(candidate);
    } catch (e) {
      console.error('addIceCandidate() failed:', e);
    }
  }

  function sendIceCandidate(candidate) {
    console.log('---sending ICE candidate ---');
    const message = JSON.stringify({ type: 'candidate', ice: candidate });
    console.log('sending candidate=' + message);
    ws.send(message);
  }

  function playVideo(element, stream) {
    element.srcObject = stream;
    // リモートストリーム更新に応じて UI も更新
    try { updateUiState(); } catch (_) {}
  }

  // 接続状況に応じて UI を更新
  function updateUiState() {
    const connectBtn = document.getElementById('btnConnect');
    const disconnectBtn = document.getElementById('btnDisconnect');
    const playBtn = document.getElementById('btnPlay');
    const sendBtn = document.getElementById('btnSend');
    const codecSelect = document.getElementById('codec');

    const hasPc = !!peerConnection;
    const hasStream = !!remoteVideo.srcObject;
    const canSend = !!(dataChannel && dataChannel.readyState === 'open');

    if (connectBtn) connectBtn.disabled = hasPc;
    if (disconnectBtn) disconnectBtn.disabled = !hasPc;
    if (playBtn) playBtn.disabled = !hasStream;
    if (sendBtn) sendBtn.disabled = !canSend;
    if (dataTextInput) dataTextInput.disabled = !canSend;
    if (codecSelect) codecSelect.disabled = hasPc; // 接続中は変更不可にする
  }

  function prepareNewConnection() {
    // 新規接続用に候補バッファ状態をリセット
    candidates = [];
    hasReceivedSdp = false;

    const peer = new RTCPeerConnection(peerConnectionConfig);
    dataChannel = peer.createDataChannel('serial');
    // DataChannel の状態に応じて UI 更新
    dataChannel.onopen = () => { console.log('DataChannel open'); updateUiState(); };
    dataChannel.onclose = () => { console.log('DataChannel close'); updateUiState(); };

    if ('ontrack' in peer) {
      if (isSafari()) {
        const tracks = [];
        peer.ontrack = (event) => {
          console.log('-- peer.ontrack()');
          tracks.push(event.track);
          // safari で動作させるために、ontrack が発火するたびに MediaStream を作成する
          const mediaStream = new MediaStream(tracks);
          playVideo(remoteVideo, mediaStream);
        };
      } else {
        const mediaStream = new MediaStream();
        playVideo(remoteVideo, mediaStream);
        peer.ontrack = (event) => {
          console.log('-- peer.ontrack()');
          mediaStream.addTrack(event.track);
        };
      }
    } else {
      peer.onaddstream = (event) => {
        console.log('-- peer.onaddstream()');
        playVideo(remoteVideo, event.stream);
      };
    }

    peer.onicecandidate = (event) => {
      console.log('-- peer.onicecandidate()');
      if (event.candidate) {
        console.log(event.candidate);
        sendIceCandidate(event.candidate);
      } else {
        console.log('empty ice event');
      }
    };

    peer.oniceconnectionstatechange = () => {
      console.log('-- peer.oniceconnectionstatechange()');
      console.log('ICE connection Status has changed to ' + peer.iceConnectionState);
      switch (peer.iceConnectionState) {
        case 'closed':
        case 'failed':
        case 'disconnected':
          break;
      }
      updateUiState();
    };

    const videoTransceiver = peer.addTransceiver('video', { direction: 'recvonly' });
    applyVideoCodecPreferences(videoTransceiver);
    peer.addTransceiver('audio', { direction: 'recvonly' });

    dataChannel.onmessage = (event) => {
      console.log('Got Data Channel Message:', new TextDecoder().decode(event.data));
    };

    return peer;
  }

  // テストページ要素がある場合に UI イベントを結線
  function bindUi() {
    const connectBtn = document.getElementById('btnConnect');
    const disconnectBtn = document.getElementById('btnDisconnect');
    const playBtn = document.getElementById('btnPlay');
    const sendBtn = document.getElementById('btnSend');
    if (connectBtn) connectBtn.addEventListener('click', () => connect());
    if (disconnectBtn) disconnectBtn.addEventListener('click', () => disconnect());
    if (playBtn) playBtn.addEventListener('click', () => play());
    if (sendBtn) sendBtn.addEventListener('click', () => sendDataChannel());
    if (dataTextInput) {
      dataTextInput.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') sendDataChannel();
      });
      // 入力変化で送信可否の見た目を即時反映
      dataTextInput.addEventListener('input', () => updateUiState());
    }
    // 初期状態反映
    updateUiState();
  }

  // コーデック選好関連の補助関数
  function getSelectedVideoCodecMime() {
    const el = document.getElementById('codec');
    if (!el) return null;
    return (el.value || '').toLowerCase() || null; // value は 'video/h264' のような完全な mimeType
  }

  function labelForMime(mime) {
    const lower = (mime || '').toLowerCase();
    const name = lower.split('/')[1] || lower;
    if (name === 'h264') return 'H264';
    if (name === 'h265' || name === 'hevc') return 'H265';
    return name.toUpperCase();
  }

  function isVideoPrimaryCodec(codec) {
    const mt = (codec && codec.mimeType ? codec.mimeType : '').toLowerCase();
    if (!mt.startsWith('video/')) return false;
    return !/(\/rtx|\/red|\/ulpfec|\/flexfec)/.test(mt);
  }

  function populateVideoCodecSelect() {
    const select = document.getElementById('codec');
    if (!select) return;
    if (typeof RTCRtpSender === 'undefined' || !RTCRtpSender.getCapabilities) return;
    const caps = RTCRtpSender.getCapabilities('video');
    if (!caps || !Array.isArray(caps.codecs)) return;

    const before = (select.value || '').toLowerCase();
    // mimeType で重複排除し、RTX/FEC 系を除外
    const seen = new Set();
    const mimes = [];
    caps.codecs.forEach(c => {
      if (!isVideoPrimaryCodec(c)) return;
      const mt = (c.mimeType || '').toLowerCase();
      if (seen.has(mt)) return;
      seen.add(mt);
      mimes.push(mt);
    });

    // いったん空にしてから再構築
    while (select.firstChild) select.removeChild(select.firstChild);
    mimes.forEach(mt => {
      const opt = document.createElement('option');
      opt.value = mt;
      opt.textContent = labelForMime(mt);
      select.appendChild(opt);
    });
    // 以前の選択があれば維持、なければ H264、なければ先頭
    const prefer = before && mimes.includes(before) ? before : (mimes.includes('video/h264') ? 'video/h264' : (mimes[0] || ''));
    if (prefer) select.value = prefer;
  }

  function applyVideoCodecPreferences(transceiver) {
    const mime = getSelectedVideoCodecMime();
    if (!mime) return;
    if (typeof RTCRtpSender === 'undefined' || !RTCRtpSender.getCapabilities) return;
    if (!transceiver || typeof transceiver.setCodecPreferences !== 'function') return;
    const caps = RTCRtpSender.getCapabilities('video');
    if (!caps || !Array.isArray(caps.codecs)) return;
    // RTX/FEC 系を除外し、一次の映像コーデックのみを使用
    const primary = caps.codecs.filter(isVideoPrimaryCodec);
    const selected = primary.filter(c => (c.mimeType || '').toLowerCase() === mime);
    const others = primary.filter(c => (c.mimeType || '').toLowerCase() !== mime);
    if (selected.length === 0) return; // 選択したコーデックが未対応
    const ordered = selected.concat(others);
    transceiver.setCodecPreferences(ordered);
    console.log('Applied codec preferences via setCodecPreferences:', mime);
  }

  function browser() {
    const ua = window.navigator.userAgent.toLocaleLowerCase();
    if (ua.indexOf('edge') !== -1) return 'edge';
    if (ua.indexOf('chrome') !== -1 && ua.indexOf('edge') === -1) return 'chrome';
    if (ua.indexOf('safari') !== -1 && ua.indexOf('chrome') === -1) return 'safari';
    if (ua.indexOf('opera') !== -1) return 'opera';
    if (ua.indexOf('firefox') !== -1) return 'firefox';
    return undefined;
  }

  function isSafari() {
    return browser() === 'safari';
  }

  function sendSdp(sessionDescription) {
    console.log('---sending sdp ---');
    const message = JSON.stringify(sessionDescription);
    console.log('sending SDP=' + message);
    ws.send(message);
  }

  async function makeOffer() {
    peerConnection = prepareNewConnection();
    updateUiState();
    try {
      const sessionDescription = await peerConnection.createOffer({
        // 受信方向は addTransceiver('recvonly') によって指定済み
      });
      console.log('createOffer() success in promise, SDP=', sessionDescription.sdp);
      await peerConnection.setLocalDescription(sessionDescription);
      console.log('setLocalDescription() success in promise');
      sendSdp(peerConnection.localDescription);
    } catch (error) {
      console.error('makeOffer() ERROR:', error);
    }
  }

  async function makeAnswer() {
    console.log('sending Answer. Creating remote session description...');
    if (!peerConnection) {
      console.error('peerConnection DOES NOT exist!');
      return;
    }
    try {
      const sessionDescription = await peerConnection.createAnswer();
      console.log('createAnswer() success in promise');
      await peerConnection.setLocalDescription(sessionDescription);
      console.log('setLocalDescription() success in promise');
      sendSdp(peerConnection.localDescription);
      drainCandidate();
    } catch (error) {
      console.error('makeAnswer() ERROR:', error);
    }
  }

  // offer SDP を受信した際の処理
  async function setOffer(sessionDescription) {
    if (peerConnection) {
      console.warn('peerConnection already exists. Replacing with a new one.');
      try { peerConnection.close(); } catch (_) {}
    }
    peerConnection = prepareNewConnection();
    updateUiState();
    try {
      await peerConnection.setRemoteDescription(sessionDescription);
      console.log('setRemoteDescription(offer) success in promise');
      await makeAnswer();
    } catch (error) {
      console.error('setRemoteDescription(offer) ERROR:', error);
    }
  }

  async function setAnswer(sessionDescription) {
    if (!peerConnection) {
      console.error('peerConnection DOES NOT exist!');
      return;
    }
    try {
      await peerConnection.setRemoteDescription(sessionDescription);
      console.log('setRemoteDescription(answer) success in promise');
      drainCandidate();
    } catch (error) {
      console.error('setRemoteDescription(answer) ERROR:', error);
    }
  }

  function cleanupVideoElement(element) {
    try {
      const stream = element.srcObject;
      if (stream && typeof stream.getTracks === 'function') {
        stream.getTracks().forEach((t) => {
          try { t.stop(); } catch (_) {}
        });
      }
    } catch (e) {
      console.warn('cleanupVideoElement: stop tracks error:', e);
    }
    element.pause();
    element.srcObject = null;
    try { updateUiState(); } catch (_) {}
  }

  // SDP 書き換えは廃止し、setCodecPreferences のみに統一

  // グローバルに公開する API（クラス未使用）
  window.connect = connect;
  window.disconnect = disconnect;
  window.play = play;
  window.sendDataChannel = sendDataChannel;
})();
