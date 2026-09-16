# E2E テストが接続の成立を検証しておらず「通っているが何も検証していない」テストが多数ある

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/add-e2e-connection-verification
- Polished: 2026-09-13

## 目的

E2E テストの多くが metrics の `"version" in data` や stats の構造だけを確認しており、実際の WebRTC 接続成立 (candidate-pair の succeeded / transport の dtlsState connected) を検証していない。WebRTC 接続が成立していなくてもパスする「通っているが検証していない」テストが多数ある。また、初回レビューで見つかったクラッシュバグ (不正 JSON、recvonly 時の /mute、DataChannel シグナリング、シリアル) は本 issue から分離され、0015-fix-signaling-json-exception / 0016-fix-local-stream-empty-crash / 0028-fix-ice-candidate-null-websocket / 0029-fix-zlib-uncompress-memory で修正済み、シリアルは 0026-fix-serial-data-unbounded-buffer として別途対応中である。だが、回帰を検出する E2E テストが不足している。テストを強化する。

## 現状

- `test/test_p2p_mode.py` の `test_with_custom_arguments` / `test_multiple_instances_concurrent` / `test_multiple_instances_different_configs` / `test_dynamic_instance_creation_and_cleanup`: `get_metrics()` の `"version" in data` のみ。P2P の実接続・メディア送受信・コマンドを検証していない
  - 同ファイルの `test_invalid_signaling_json_does_not_crash` はシグナリング JSON の不正入力でプロセスが落ちないことを検証するが、接続成立やメディア送受信は検証しない
- `test/test_sora_mode.py` の `test_metrics_endpoint_returns_200` / `test_metrics_endpoint_response_structure`: stats の構造のみで `candidate-pair` の succeeded / `transport.dtlsState` を確認していない
- `test/test_ayame_mode.py` の `test_ayame_mode_basic` / `test_ayame_mode_with_client_id` / `test_ayame_mode_with_video_settings` / `test_ayame_mode_with_audio_settings`: `assert "version" in data` のみのテスト
- `test/test_sora_mode_sendonly_recvonly.py` の `test_multiple_sendonly_clients`: metrics の非 None のみ
- `test/momo.py` の `_wait_for_startup`: metrics エンドポイントの応答のみで接続成立を待たない (Sora モードは stats が空でないことだけを確認する)
- 不正 JSON の E2E テストは `test/test_p2p_mode.py` の `test_invalid_signaling_json_does_not_crash` のみ (Sora / Ayame モードのシグナリング不正 JSON テストは無い)。recvonly 時の `/mute`・DataChannel シグナリング・`--serial` のテストは存在しない
- `test/test_sora_mode_raspberry_pi.py` の `test_sora_sendonly_recvonly_pair`: 無期限スキップ (`@pytest.mark.skipif(reason="上手く動作しないため一時的にスキップ")`)

## 設計方針

- 接続成立の判定を、`candidate-pair` の `state == "succeeded"` と `transport.dtlsState == "connected"` の検証に強化する。既存の `Momo.wait_for_connection()` (`test/momo.py`) は `transport.dtlsState` と `transport.iceState` を確認するため、`candidate-pair` の `state == "succeeded"` を追加で確認するよう拡張し、上記の弱いテストから利用する
- recvonly 時の `/mute`・DataChannel シグナリング・`--serial` の E2E テストを追加する (クラッシュバグ自体は各 issue で修正済みのため、回帰検出が目的)
  - recvonly 時の `/mute` は 0068-add-recvonly-receive-control で機能追加が見込まれるため、現状の `400 Bad Request` 応答とクラッシュしないことの回帰確認に留める
  - `--serial` はモック・スタブを使わず、socat 等で生成する実シリアルデバイス (PTY ペア) を利用する
- Raspberry Pi の sendonly/recvonly ペアテストのスキップを解除し、失敗理由を特定して修正する
- `test/momo.py` の `_wait_for_startup` に接続成立待ちを追加する

## 完了条件

- 接続が成立していない場合にテストが失敗する (偽陽性が無くなる)
- 上記の回帰を検出するテストが追加されている
- 全 E2E テストが CI でパスする

## 解決方法

未着手 (PR 作成後に追記する)
