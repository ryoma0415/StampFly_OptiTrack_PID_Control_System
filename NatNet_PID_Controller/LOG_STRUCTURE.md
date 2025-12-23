# StampFly ホバリングログ形式

この文書は `python_controller/hovering_controller.py` と `python_controller/circling_controller.py` が生成するCSVの列構成を説明します。ログは制御開始時に `flight_logs/` へ `log_YYYYMMDD_HHMMSS.csv` の形式で保存されます。

## 座標系と単位

* 位置(`pos_*`, `raw_pos_*`, `rb_pos_*`)はMotive座標(X, Y, Z)をドローン座標系へ変換した後の値で、単位はmです。デフォルトはZ→X、X→Y、Y→Zですが、`config.json` の `coordinate_transform` で変更できます。
* `_rad`はラジアン、`_deg`は度です。
* 時間系(`loop_time_ms`, `feedback_latency_ms`, `feedback_age_ms`, `frame_dt_ms`)はミリ秒です。
* 0/1のフラグは文字列で記録され、値が未取得のときは空文字になります。

## 列リファレンス

### セッション/タイミング

| 列 | 型 | 説明 |
| --- | --- | --- |
| `timestamp` | ISO8601文字列 | 行が記録された時刻(ローカル時刻)。 |
| `elapsed_time` | float (s) | ログ開始からの経過秒。 |
| `frame_number` | int | 最新のNatNetフレーム番号。 |
| `marker_count` | int | このフレームで有効と判断されたマーカー数。`data_source` が `rigid_body` の場合はリジッドボディのマーカー数(取得できた場合)、`markers` の場合はラベル付きマーカー数。 |
| `loop_time_ms` | float (ms) | 制御ループ1回の実行時間。 |
| `frame_dt_ms` | float (ms) | 連続するNatNetフレーム間の時間差(PID更新に使用した値)。 |
| `command_sequence` | int (文字列) | 送信した角度コマンドのシーケンス番号。初回送信前は空。位置データがないループでは最後に送信した番号が再記録されることがあります。 |
| `feedback_sequence` | int (文字列) | 最新フィードバックに含まれるシーケンス番号。行の`command_sequence`と一致するとは限りません。 |
| `feedback_latency_ms` | float (ms) | フィードバック受信時にpendingコマンドが見つかった場合の往復遅延。未マッチのときは空。 |
| `feedback_age_ms` | float (ms) | 最新フィードバックの受信時刻からこの行の記録時刻までの経過時間(古いフィードバック検知用)。 |

### 位置と誤差

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `pos_x`, `pos_y`, `pos_z` | float | m | 制御に使用したフィルタ後の位置。 |
| `raw_pos_x`, `raw_pos_y`, `raw_pos_z` | float | m | フィルタへ入力した生位置(リジッドボディまたはマーカー重心)。 |
| `error_x`, `error_y` | float | m | 有効な目標位置に対するXY誤差。 |
| `target_x`, `target_y` | float | m | 誤差計算に使ったXY平面の目標位置。 |

### 指令姿勢とPID成分

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `roll_ref_rad`, `pitch_ref_rad` | float | rad | ESP32リレーに送信した角度指令。 |
| `roll_ref_deg`, `pitch_ref_deg` | float | deg | 上記の度数表記。 |
| `pid_x_p`, `pid_x_i`, `pid_x_d` | float | rad相当 | X軸PID(ロール)のP/I/D成分。 |
| `pid_y_p`, `pid_y_i`, `pid_y_d` | float | rad相当 | Y軸PID(ピッチ)のP/I/D成分。 |
| `send_success` | 0/1 | - | シリアル書き込み成功フラグ(1で送信成功)。 |
| `control_active` | 0/1 | - | 外側ループの制御が有効かどうか。 |

### StampFlyからのフィードバック

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `feedback_roll_rad`, `feedback_pitch_rad` | float | rad | StampFlyから受信した最新の適用角。未受信時は空。 |
| `feedback_match` | 0/1 | - | フィードバック受信時にpendingコマンドと一致した場合に1。行の`command_sequence`と一致することを保証しません。 |
| `feedback_delta_roll`, `feedback_delta_pitch` | float | rad | 最新フィードバックとこの行の指令角の差(フィードバック - 指令)。シーケンスが一致しない場合もあります。 |

### フィルタ状態とデータ由来

| 列 | 型 | 説明 |
| --- | --- | --- |
| `is_outlier` | 0/1 | フィルタが外れ値と判定したかどうか。 |
| `used_prediction` | 0/1 | 生データではなく予測位置を使用したかどうか。 |
| `confidence` | float (0-1) | フィルタの信頼度スコア(制御データの有効/無効判定に使用)。 |
| `consecutive_outliers` | int | 連続外れ値フレーム数。 |
| `data_valid` | 0/1 | PID更新に有効と判断されたかどうか(外れ値/トラッキング/フレーム遅延を考慮)。 |
| `data_source` | string | データ由来: `"rigid_body"`, `"markers"`, `"none"`など。 |
| `filter_threshold` | float (m) | 外れ値判定に使った動的距離しきい値(未取得時は空)。 |
| `tracking_valid` | 0/1 | データソースのトラッキング有効判定。`rigid_body`はMotiveの`tracking_valid`、`markers`は`marker_count>=3`。フィルタ結果がない行では`data_valid`相当。 |

### リジッドボディ診断

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `rb_error` | float | - | NatNetが報告するリジッドボディの解法エラー(小さいほど良い)。 |
| `rb_marker_count` | int | count | リジッドボディ解法に寄与したマーカー数。 |
| `rb_pos_x`, `rb_pos_y`, `rb_pos_z` | float | m | 最新のリジッドボディ位置(ドローン座標系)。取得できない場合は空。 |
| `rb_qx`, `rb_qy`, `rb_qz`, `rb_qw` | float | - | Motiveの姿勢クォータニオン。 |
| `rb_roll_deg`, `rb_pitch_deg`, `rb_yaw_deg` | float | deg | クォータニオンから算出したオイラー角。 |

## 補足

* 行は制御周期(名目100Hz)で出力されます。OptiTrackデータが古い場合、最後の指令を保持しつつ`data_valid`が0になります。
* シーケンス番号は2^32でロールオーバーします。Pythonプロセスが再起動すると`command_sequence`は0から再開します。
* フィードバック関連の列は、ESP32リレーが有効な`CMD_FEEDBACK`を受信し、バイナリ`F`フレームとして転送するまで空です。
* 追加の警告や詳細情報はコンソールに出力されますがCSVには含まれません。`flight_logs/`と併せて端末出力を確認してください。

このリファレンスで、生成されたフライトログの可視化・解析ができるはずです。
