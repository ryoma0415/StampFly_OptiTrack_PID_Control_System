# StampFly OptiTrack PID Control System

OptiTrack/Motive の位置計測を使って M5StampFly（StampS3 搭載マイクロドローン）をホバリングさせるための制御一式です。  
PC 側の NatNet PID（XY 位置）＋ ESP32 リレー（USB Serial → ESP-NOW）＋ 機体側の姿勢/高度制御（400Hz）を連携させます。

## システム全体像

```
OptiTrack/Motive
    │ NatNet (位置/姿勢)
    ▼
Python (filter + XY PID, 100Hz)
    │ USB Serial (A/F バイナリフレーム)
    ▼
ESP32 Relay (esp32_relay.ino)
    │ ESP-NOW
    ▼
StampS3 / M5StampFly Firmware
    │ 姿勢/高度制御 + モータミキシング
    ▼
Motors
```

### 主な特徴
- 位置外乱に強い外部 XY PID（D フィルタ・I 条件更新・異常時減衰）
- OptiTrack リジッドボディ優先・マーカー重心フォールバック
- 位置外れ値検出＋予測補間フィルタ
- 角度指令のシーケンス番号付き送受信（ESP-NOW 経由）
- CSV ログ出力（ログ構造は `NatNet_PID_Controller/LOG_STRUCTURE.md` 参照）

## リポジトリ構成

- `M5StampFly/`  
  StampFly 本体ファームウェア（PlatformIO）。姿勢・高度の内側制御、ESP-NOW 受信、角度指令反映。
- `NatNet_PID_Controller/`  
  OptiTrack/NatNet 連携の Python 制御系。ホバリング/サークル軌道制御、ログ出力。
- `esp32_relay/`  
  PC → ESP32 → ESP-NOW への中継スケッチ。シリアルバイナリフレームと start/stop 文字列コマンド対応。

## 必要なハードウェア

- M5StampFly (StampS3 搭載機体)
- OptiTrack + Motive（マーカー構成 or リジッドボディ設定）
- ESP32-WROOM-32E（リレー用途）
- USB 接続可能な PC

## 必要なソフトウェア

- Python 3.x
- PlatformIO（M5StampFly ビルド用）
- Arduino IDE/CLI または PlatformIO（ESP32 リレー書き込み用）
- OptiTrack Motive（NatNet 配信有効化）

## セットアップ手順

### 1) M5StampFly ファームウェア

PlatformIO で `M5StampFly/` を開き、ビルド・書き込みします。

```bash
pio run -d M5StampFly
pio run -d M5StampFly -t upload
```

既存バイナリは `M5StampFly/firmware/` にあります（例: `stampfly_firmware_v1.1.0_20240812.bin`）。

### 2) ESP32 リレー（esp32_relay.ino）

- `esp32_relay/esp32_relay.ino` を書き込み。
- 宛先の StampS3 MAC を `broadcastAddress` に設定。
- 逆方向フィードバック用に、機体側の `M5StampFly/src/main.cpp` の `relay_mac_address` を ESP32 の MAC に合わせます。
- 通信速度は 115200 baud。

### 3) OptiTrack/Motive 設定

Motive 側で NatNet ストリーミングを有効化します。

- `Edit -> Settings -> Streaming`
  - `Broadcast Frame Data` を有効化
  - `Local Interface` に使用するネットワークインターフェースを選択

リジッドボディ ID は `NatNet_PID_Controller/config.json` の `rigid_body_id` で指定します。

### 4) Python 環境

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r NatNet_PID_Controller/requirements.txt
```

## 実行方法

### ホバリング制御

```bash
python NatNet_PID_Controller/python_controller/hovering_controller.py
```

起動後、以下の対話コマンドを使用します。

- `start` : 離陸してホバリング制御開始
- `stop`  : 着陸
- `exit`  : 終了

### サークル軌道（外周追従）

```bash
python NatNet_PID_Controller/python_controller/circling_controller.py
```

`config.json` の `trajectory` 設定に従って円軌道を生成します。

### NatNet 受信確認（リジッドボディ姿勢）

```bash
python NatNet_PID_Controller/rigid_body_pose_stream.py --server 127.0.0.1 --client 127.0.0.1
```

## 設定ファイル（`NatNet_PID_Controller/config.json`）

- `pid`  
  - `x`, `y`: PID ゲイン（X/Y 軸）  
  - `output_limit`: 出力制限（rad、例: ±0.087）  
  - `d_filter_alpha`, `i_decay_rate`, `i_update_threshold`, `enable_i_control`
- `target_position`  
  - 原点ホバリングの目標座標（m）
- `trajectory`  
  - `radius`, `period_sec`, `start_hold_sec`, `clockwise`（サークル制御用）
- `rigid_body_id`  
  - Motive のリジッドボディ ID
- `control`  
  - `rate_hz`: 制御周期（100Hz）  
  - `frame_hold_ms`: フレーム欠損時に前回指令を保持する時間  
  - `confidence_zero_threshold`: 低信頼時の制御抑制しきい値
- `filter`  
  - `window_size`, `outlier_threshold`, `velocity_window`, `enable_prediction`, `max_outlier_threshold`
- `coordinate_transform`  
  - Motive 座標 → ドローン座標の軸割当（例: `x <- z`, `y <- x`, `z <- y`）

## ログ出力

- `NatNet_PID_Controller/flight_logs/log_YYYYMMDD_HHMMSS.csv` に保存
- 列定義は `NatNet_PID_Controller/LOG_STRUCTURE.md` を参照

## シリアル/ESP-NOW プロトコル概要

- 文字列コマンド: `start`, `stop`（改行終端）
- 角度指令フレーム（PC → Relay）
  - `A` + `uint32 sequence` + `float roll` + `float pitch` + `checksum`
- フィードバックフレーム（StampS3 → Relay → PC）
  - `F` + `uint32 sequence` + `float roll` + `float pitch` + `checksum`
- `checksum` は payload の 1byte 合計（`sum(payload) & 0xFF`）

## 機体側制御（M5StampFly）

- 400Hz の制御ループ
- 角度 PID（外側）→ 角速度 PID（内側）のカスケード
- ToF + 加速度のカルマンフィルタで高度推定
- 自動離陸/ホバー/着陸の状態遷移
- 外部からのロール/ピッチ指令を `ESP-NOW` で受信

詳細は `M5StampFly/CONTROL_SYSTEM_PID.md` を参照してください。

## テスト/ユーティリティ

- `NatNet_PID_Controller/test_improved_system.py`  
  フィルタ・PID の単体テスト
- `NatNet_PID_Controller/python_controller/test_hovering.py`  
  位置取得のみテスト or 通常ホバリング試験
- `NatNet_PID_Controller/rigid_body_pose_stream.py`  
  NatNet 受信デバッグ（リジッドボディ姿勢）

## トラブルシューティング

- Motive から位置が来ない  
  - Streaming 設定、`rigid_body_id`、マーカー認識数を確認
- 角度指令が届かない  
  - ESP32 の MAC 設定（relay/firmware 両方）を確認
- シリアル接続に失敗  
  - `hovering_controller.py` の `serial_port` を使用環境に合わせる
- 振動/発散  
  - `config.json` の PID ゲイン/出力制限を調整

## ライセンス

- ファームウェア部分は `M5StampFly/LICENSE` を参照（MIT License）
- そのほかのディレクトリは明記されたライセンスに従います

## 参考/謝辞

- ベースプロジェクト: https://github.com/M5Fly-kanazawa/StampFly2024June
- M5StampFly 製品情報: https://docs.m5stack.com/en/app/Stamp%20Fly
