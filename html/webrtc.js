const remoteVideo = document.getElementById('remote_video');
let localStream = null;
let peerConnection = null;
let candidates = [];
let hasReceivedSdp = false;

const wsUrl = 'ws://' + location.host + '/ws';
const ws = new WebSocket(wsUrl);
ws.onopen = function (evt) {
    console.log('ws open()');
};
ws.onerror = function (err) {
    console.error('ws onerror() ERR:', err);
};
ws.onmessage = function (evt) {
    console.log('ws onmessage() data:', evt.data);
    const message = JSON.parse(evt.data);
    if (message.type === 'offer') {
        console.log('Received offer ...');
        const offer = new RTCSessionDescription(message);
        setOffer(offer);
    }
    else if (message.type === 'answer') {
        console.log('Received answer ...');
        const answer = new RTCSessionDescription(message);
        setAnswer(answer);
    }
    else if (message.type === 'candidate') {
        console.log('Received ICE candidate ...');
        const candidate = new RTCIceCandidate(message.ice);
        console.log(candidate);
        if (hasReceivedSdp) {
            addIceCandidate(candidate);
        } else {
            candidates.push(candidate);
        }
    }
    else if (message.type === 'close') {
        console.log('peer is closed ...');
    }
};

function drainCandidate() {
    hasReceivedSdp = true;
    candidates.forEach(function (candidate, index, array) {
        addIceCandidate(candidate);
    });
    candidates = [];
}

function addIceCandidate(candidate) {
    if (peerConnection) {
        peerConnection.addIceCandidate(candidate);
    }
    else {
        console.error('PeerConnection not exist!');
        return;
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
}

function prepareNewConnection() {
    const peer = new RTCPeerConnection({ "iceServers": [{ "urls": "stun:stun.l.google.com:19302" }] });

    if ('ontrack' in peer) {
        let mediaStream = new MediaStream();
        playVideo(remoteVideo, mediaStream);
        peer.ontrack = function (event) {
            console.log('-- peer.ontrack()');
            mediaStream.addTrack(event.track);
        };
    }
    else {
        peer.onaddstream = function (event) {
            console.log('-- peer.onaddstream()');
            playVideo(remoteVideo, event.stream);
        };
    }

    peer.onicecandidate = function (evt) {
        if (evt.candidate) {
            console.log(evt.candidate);
            sendIceCandidate(evt.candidate);
        } else {
            console.log('empty ice event');
        }
    };

    peer.oniceconnectionstatechange = function () {
        console.log('ICE connection Status has changed to ' + peer.iceConnectionState);
        switch (peer.iceConnectionState) {
            case 'closed':
            case 'failed':
            case 'dissconnected':
                break;
        }
    };

    return peer;
}

function sendSdp(sessionDescription) {
    console.log('---sending sdp ---');
    const message = JSON.stringify(sessionDescription);
    console.log('sending SDP=' + message);
    ws.send(message);
}

function connect() {
    if (!peerConnection) {
        console.log('make Offer');
        makeOffer();
    }
    else {
        console.warn('peer already exist.');
    }
}

function makeOffer() {
    peerConnection = prepareNewConnection();
    peerConnection.createOffer({
        'offerToReceiveAudio': true,
        'offerToReceiveVideo': true
    })
        .then(function (sessionDescription) {
            console.log('createOffer() succsess in promise');
            switch (document.getElementById("codec").selectedIndex) {
                case 0:
                    sessionDescription.sdp = removeCodec(sessionDescription.sdp, "VP8");
                    sessionDescription.sdp = removeCodec(sessionDescription.sdp, "VP9");
                    break;
                case 1:
                    sessionDescription.sdp = removeCodec(sessionDescription.sdp, "H264");
                    sessionDescription.sdp = removeCodec(sessionDescription.sdp, "VP9");
                    break;
                case 2:
                    sessionDescription.sdp = removeCodec(sessionDescription.sdp, "H264");
                    sessionDescription.sdp = removeCodec(sessionDescription.sdp, "VP8");
                    break;
            }
            return peerConnection.setLocalDescription(sessionDescription);
        }).then(function () {
            console.log('setLocalDescription() succsess in promise');
            sendSdp(peerConnection.localDescription);
        }).catch(function (err) {
            console.error(err);
        });
}

function makeAnswer() {
    console.log('sending Answer. Creating remote session description...');
    if (!peerConnection) {
        console.error('peerConnection NOT exist!');
        return;
    }
    peerConnection.createAnswer()
        .then(function (sessionDescription) {
            console.log('createAnswer() succsess in promise');
            return peerConnection.setLocalDescription(sessionDescription);
        }).then(function () {
            console.log('setLocalDescription() succsess in promise');
            sendSdp(peerConnection.localDescription);
            drainCandidate();
        }).catch(function (err) {
            console.error(err);
        });
}

function setOffer(sessionDescription) {
    if (peerConnection) {
        console.error('peerConnection alreay exist!');
    }
    peerConnection = prepareNewConnection();
    peerConnection.onnegotiationneeded = function () {
        peerConnection.setRemoteDescription(sessionDescription)
            .then(function () {
                console.log('setRemoteDescription(offer) succsess in promise');
                makeAnswer();
            }).catch(function (err) {
                console.error('setRemoteDescription(offer) ERROR: ', err);
            });
    }
}

function setAnswer(sessionDescription) {
    if (!peerConnection) {
        console.error('peerConnection NOT exist!');
        return;
    }
    peerConnection.setRemoteDescription(sessionDescription)
        .then(function () {
            console.log('setRemoteDescription(answer) succsess in promise');
            drainCandidate();
        }).catch(function (err) {
            console.error('setRemoteDescription(answer) ERROR: ', err);
        });
}

function disconnect() {
    if (peerConnection) {
        if (peerConnection.iceConnectionState !== 'closed') {
            peerConnection.close();
            peerConnection = null;
            if (ws && ws.readyState === 1) {
                const message = JSON.stringify({ type: 'close' });
                ws.send(message);
            }
            console.log('sending close message');
            cleanupVideoElement(remoteVideo);
            return;
        }
    }
    console.log('peerConnection is closed.');
}

function cleanupVideoElement(element) {
    element.pause();
    element.srcObject = null;
}

function getOffer() {
    initiator = false;
    createPeerConnection();
    sendXHR(
        ".GetOffer",
        JSON.stringify(peer_connection.localDescription),
        function (respnse) {
            peer_connection.setRemoteDescription(
                new RTCSessionDescription(respnse),
                function () {
                    peer_connection.createAnswer(
                        function (answer) {
                            peer_connection.setLocalDescription(answer);
                        }, function (e) { });
                }, function (e) {
                    console.error(e);
                });
        }, true);
}

function removeCodec(orgsdp, codec) {
    var internalFunc = function (sdp) {
        var codecre = new RegExp('(a=rtpmap:(\\d*) ' + codec + '\/90000\\r\\n)');
        var rtpmaps = sdp.match(codecre);
        if (rtpmaps == null || rtpmaps.length <= 2) {
            return sdp;
        }
        var rtpmap = rtpmaps[2];
        var modsdp = sdp.replace(codecre, "");

        var rtcpre = new RegExp('(a=rtcp-fb:' + rtpmap + '.*\r\n)', 'g');
        modsdp = modsdp.replace(rtcpre, "");

        var fmtpre = new RegExp('(a=fmtp:' + rtpmap + '.*\r\n)', 'g');
        modsdp = modsdp.replace(fmtpre, "");

        var aptpre = new RegExp('(a=fmtp:(\\d*) apt=' + rtpmap + '\\r\\n)');
        var aptmaps = modsdp.match(aptpre);
        var fmtpmap = "";
        if (aptmaps != null && aptmaps.length >= 3) {
            fmtpmap = aptmaps[2];
            modsdp = modsdp.replace(aptpre, "");

            var rtppre = new RegExp('(a=rtpmap:' + fmtpmap + '.*\r\n)', 'g');
            modsdp = modsdp.replace(rtppre, "");
        }

        var videore = /(m=video.*\r\n)/;
        var videolines = modsdp.match(videore);
        if (videolines != null) {
            //If many m=video are found in SDP, this program doesn't work.
            var videoline = videolines[0].substring(0, videolines[0].length - 2);
            var videoelem = videoline.split(" ");
            var modvideoline = videoelem[0];
            for (var i = 1; i < videoelem.length; i++) {
                if (videoelem[i] == rtpmap || videoelem[i] == fmtpmap) {
                    continue;
                }
                modvideoline += " " + videoelem[i];
            }
            modvideoline += "\r\n";
            modsdp = modsdp.replace(videore, modvideoline);
        }
        return internalFunc(modsdp);
    };
    return internalFunc(orgsdp);
}
