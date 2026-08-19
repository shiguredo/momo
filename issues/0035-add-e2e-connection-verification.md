# E2E テストが接続の成立を検証しておらず「通っているが何も検証していない」テストが多数ある

- Created: 2026-08-19
- Completed: {YYYY-MM-DD}
- Branch: feature/add-e2e-connection-verification
- Polished: {YYYY-MM-DD}

## 目的

E2E テストの多くが metrics の `"version" in data` や stats の構造だけを確認しており、実際の WebRTC 接続成立 (candidate-pair の succeeded / transport の dtlsState connected) を検証していない。サーバが停止していてもパスする「通っているが検証していない」テストが多数ある。また、1 周目のレビューで見つかったクラッシュバグ (不正 JSON、recvonly 時の /mute、DataChannel シグナリング、シリアル) を検出するテストが存在しない。テストを強化する。

## 現状

- `test/test_p2p_mode.py` 全体: `get_metrics()` の `"version" in data` のみ。P2P の実接続・メディア送受信・コマンドを検証していない
- `test/test_sora_mode.py` (28-57 行): stats の構造のみで `candidate-pair` succeeded / `transport.dtlsState` を確認していない
- `test/test_ayame_mode.py` (44-96, 197-212 行): `assert "version" in data` のみのテストが複数
- `test/test_sora_mode_sendonly_recvonly.py` (200-231 行): metrics の非 None のみ
- `test/momo.py` の `_wait_for_startup` は metrics エンドポイントの応答のみで接続成立を待たない
- 不正 JSON・`/mute` (recvonly)・DataChannel シグナリング・`--serial` のテストが 1 本も存在しない
- `test/test_sora_mode_raspberry_pi.py` (354 行): sendonly/recvonly ペアテストが無期限スキップ (`@pytest.mark.skipif(reason="上手く動作しないため一時的にスキップ")`)

## 設計方針

- 接続成立の判定を、`candidate-pair` の `state == "succeeded"` と `transport.dtlsState == "connected"` の検証に強化する
- 不正 JSON・recvonly 時の `/mute`・DataChannel シグナリング・シリアルの E2E テストを追加する
- Raspberry Pi の sendonly/recvonly ペアテストのスキップを解除し、失敗理由を特定して修正する
- `test/momo.py` の `_wait_for_startup` に接続成立待ちを追加する

## 完了条件

- 接続が成立していない場合にテストが失敗する (偽陽性が無くなる)
- 上記のクラッシュバグを検出するテストが追加されている
- 全 E2E テストが CI でパスする

## 解決方法

未着手 (PR 作成後に追記する)
