#!/usr/bin/env python3
"""
Optitrack原点ホバリング制御プログラム
NatNetでドローン位置を取得し、PID制御で角度指令を生成してESP32へ送信
"""

import serial
import struct
import time
import threading
import sys
import os
import json
import copy
import csv
import atexit
from datetime import datetime
from math import asin, atan2, copysign, degrees, pi

# 親ディレクトリのパスを追加（NatNetClient等のインポート用）
# 現在のプロジェクト内のNatNet_Controlディレクトリを参照
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, parent_dir)

from NatNetClient import NatNetClient
from pid_controller import XYPIDController
from position_filter import PositionFilter

ANGLE_HEADER = b'A'
FEEDBACK_HEADER = b'F'
FEEDBACK_FRAME_SIZE = 1 + 4 + 4 + 4 + 1  # header + sequence + roll + pitch + checksum

DEFAULT_CONFIG = {
    "pid": {
        "x": {"kp": 0.139, "ki": 0.020, "kd": 0.204},
        "y": {"kp": -0.139, "ki": -0.020, "kd": -0.204},
        "output_limit": [-0.087, 0.087],
        "d_filter_alpha": 0.6,
        "i_decay_rate": 0.98,
        "i_update_threshold": 0.3,
        "enable_i_control": True
    },
    "target_position": {"x": 0.0, "y": 0.0},
    "rigid_body_id": 1,
    "control": {
        "rate_hz": 100,
        "frame_hold_ms": 300,
        "confidence_zero_threshold": 0.2,
        "confidence_scale_threshold": 0.99
    },
    "filter": {
        "window_size": 5,
        "outlier_threshold": 0.1,
        "velocity_window": 3,
        "enable_prediction": True,
        "max_outlier_threshold": 0.4
    },
    "coordinate_transform": {
        "x": {"axis": "z", "sign": 1},
        "y": {"axis": "x", "sign": 1},
        "z": {"axis": "y", "sign": 1}
    }
}

class HoveringController:
    """Optitrackを使用した原点ホバリング制御クラス"""
    
    def __init__(self, serial_port=None, baudrate=115200, config_path=None):
        """
        初期化
        Args:
            serial_port: ESP32と接続するシリアルポート
            baudrate: ボーレート
            config_path: JSON設定ファイルのパス
        """
        self.config_path = config_path or os.path.join(parent_dir, "config.json")
        self.config = self.load_config(self.config_path)

        control_config = self.config.get("control", {})
        self.control_rate_hz = float(control_config.get("rate_hz", 100))
        self.control_period = 1.0 / self.control_rate_hz if self.control_rate_hz > 0 else 0.01
        hold_ms = float(control_config.get("frame_hold_ms", 300))
        self.frame_hold_timeout = max(0.0, hold_ms / 1000.0)
        self.confidence_zero_threshold = float(
            control_config.get("confidence_zero_threshold", 0.2)
        )
        self.confidence_scale_threshold = float(
            control_config.get("confidence_scale_threshold", 0.99)
        )

        self.axis_map = self.parse_coordinate_transform(
            self.config.get("coordinate_transform", {})
        )

        # シリアル通信設定
        # OSに応じてデフォルトポートを設定
        if serial_port is None:
            import platform
            if platform.system() == 'Windows':
                self.serial_port = "COM3"  # Windowsのデフォルト
            elif platform.system() == 'Darwin':  # macOS
                self.serial_port = "/dev/cu.usbserial-1110"
            else:  # Linux
                self.serial_port = "/dev/ttyUSB0"
        else:
            self.serial_port = serial_port
        
        self.baudrate = baudrate
        self.ser = None
        
        # 制御状態
        self.is_flying = False
        self.control_active = False
        self.shutdown_flag = False
        
        # ドローン位置（最新値を保持）
        self.current_position = None
        self.last_valid_position = None
        self.position_lock = threading.Lock()

        # フレーム時間管理
        self.last_frame_time_monotonic = None
        self.last_frame_dt = None
        self.last_frame_wall_time = None
        self.last_roll_ref = 0.0
        self.last_pitch_ref = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_control_confidence = 0.0
        self.last_control_data_valid = False
        
        # PIDコントローラ（拡張版：D項フィルタとI項制御付き）
        pid_config = self.config.get("pid", {})
        pid_x = pid_config.get("x", {})
        pid_y = pid_config.get("y", {})
        output_limit_cfg = pid_config.get("output_limit", (-0.087, 0.087))
        output_limit = None if output_limit_cfg is None else tuple(output_limit_cfg)
        self.pid_controller = XYPIDController(
            kp_x=float(pid_x.get("kp", 0.139)),
            ki_x=float(pid_x.get("ki", 0.020)),
            kd_x=float(pid_x.get("kd", 0.204)),
            kp_y=float(pid_y.get("kp", -0.139)),
            ki_y=float(pid_y.get("ki", -0.020)),
            kd_y=float(pid_y.get("kd", -0.204)),
            output_limit=output_limit,
            d_filter_alpha=float(pid_config.get("d_filter_alpha", 0.6)),
            i_decay_rate=float(pid_config.get("i_decay_rate", 0.98)),
            i_update_threshold=float(pid_config.get("i_update_threshold", 0.3)),
            enable_i_control=bool(pid_config.get("enable_i_control", True))
        )

        # 位置フィルタ（異常値除去と平滑化）
        filter_config = self.config.get("filter", {})
        self.position_filter = PositionFilter(
            window_size=int(filter_config.get("window_size", 5)),
            outlier_threshold=float(filter_config.get("outlier_threshold", 0.1)),
            velocity_window=int(filter_config.get("velocity_window", 3)),
            enable_prediction=bool(filter_config.get("enable_prediction", True)),
            max_outlier_threshold=float(filter_config.get("max_outlier_threshold", 0.4))
        )
        self.rigid_body_id = int(self.config.get("rigid_body_id", 1))
        self.last_rigid_body_pose = None
        self.last_data_source = "markers"
        self.last_filter_result = None
        
        # 目標位置（原点）
        target_position = self.config.get("target_position", {})
        self.target_position = (
            float(target_position.get("x", 0.0)),
            float(target_position.get("y", 0.0))
        )
        self.last_target_position = self.target_position
        
        # NatNetクライアント
        self.natnet_client = None
        
        # 制御スレッド
        self.control_thread = None
        
        # 統計情報
        self.packets_sent = 0
        self.command_sequence = 0
        self.last_command_sequence = None
        self.pending_commands = {}
        self.feedback_lock = threading.Lock()
        self.latest_feedback = None
        self.last_feedback_sequence = None
        self.feedback_thread = None
        self.feedback_stop_event = threading.Event()
        self.serial_buffer = bytearray()
        self.serial_text_buffer = bytearray()
        self.feedback_stats = {
            'received': 0,
            'invalid': 0
        }
        self.last_send_time = 0
        
        # データロギング関連
        self.log_file = None
        self.csv_writer = None
        self.current_frame_number = 0
        self.current_marker_count = 0
        self.log_start_time = None

        # フィルタリング関連の統計
        self.filter_stats = {
            'outliers_detected': 0,
            'predictions_used': 0,
            'total_frames': 0
        }
        
        # atexit登録（プログラム終了時にファイルを確実にクローズ）
        atexit.register(self.close_log_file)

    def load_config(self, config_path):
        """JSON設定ファイルを読み込み、デフォルトにマージ"""
        config = copy.deepcopy(DEFAULT_CONFIG)
        if not config_path:
            return config

        if not os.path.exists(config_path):
            print(f"[Config] Not found: {config_path}. Using defaults.")
            return config

        try:
            with open(config_path, 'r', encoding='utf-8') as file_handle:
                loaded = json.load(file_handle)
        except Exception as exc:
            print(f"[Config] Failed to load {config_path}: {exc}. Using defaults.")
            return config

        if isinstance(loaded, dict):
            if "liquid_id" in loaded and "rigid_body_id" not in loaded:
                loaded["rigid_body_id"] = loaded.pop("liquid_id")
            self.deep_update_config(config, loaded)
        else:
            print(f"[Config] Invalid format in {config_path}. Using defaults.")

        return config

    def deep_update_config(self, base, updates):
        """ネストされたdictを再帰的に更新"""
        for key, value in updates.items():
            if isinstance(value, dict) and isinstance(base.get(key), dict):
                self.deep_update_config(base[key], value)
            else:
                base[key] = value

    def parse_coordinate_transform(self, transform_config):
        """座標変換設定を解析して軸マップを作成"""
        default_map = DEFAULT_CONFIG.get("coordinate_transform", {})
        axis_index = {"x": 0, "y": 1, "z": 2}
        resolved = {}

        for axis in ("x", "y", "z"):
            axis_cfg = transform_config.get(axis, {}) if isinstance(transform_config, dict) else {}
            default_axis_cfg = default_map.get(axis, {})
            src_axis = axis_cfg.get("axis", default_axis_cfg.get("axis", axis))
            sign = axis_cfg.get("sign", default_axis_cfg.get("sign", 1))

            if src_axis not in axis_index:
                print(f"[Config] Invalid axis '{src_axis}' for {axis}; using default.")
                src_axis = default_axis_cfg.get("axis", axis)

            try:
                sign_value = float(sign)
            except (TypeError, ValueError):
                print(f"[Config] Invalid sign '{sign}' for {axis}; using default.")
                sign_value = float(default_axis_cfg.get("sign", 1))

            sign = -1 if sign_value < 0 else 1
            resolved[axis] = (axis_index[src_axis], sign)

        return resolved

    def get_target_position(self, frame_time_monotonic=None):
        """現在の目標位置を取得（デフォルトは固定目標）"""
        return self.target_position

    def get_target_description(self):
        """目標の説明文を返す（表示用）"""
        return f"目標位置: ({self.target_position[0]:.3f}, {self.target_position[1]:.3f}) m"

    def on_control_start(self):
        """制御開始時のフック（サブクラスで使用）"""
        return

    def get_active_mode_message(self):
        """制御中の表示文を返す（表示用）"""
        return "原点ホバリング中... (stopで着陸)"
        
    def close_log_file(self):
        """ログファイルを安全にクローズ"""
        if self.log_file:
            try:
                self.log_file.close()
                print(f"✓ ログファイルを保存しました")
            except:
                pass
            self.log_file = None
            self.csv_writer = None
    
    def start_logging(self):
        """新規ログファイルを作成して記録開始"""
        # タイムスタンプ付きファイル名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = "flight_logs"
        
        # ログディレクトリ作成
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            
        filename = os.path.join(log_dir, f"log_{timestamp}.csv")
        
        try:
            # 既存のファイルをクローズ
            self.close_log_file()
            
            # 新規ファイルオープン（行バッファリングで即座に書き込み）
            self.log_file = open(filename, 'w', newline='', buffering=1)
            self.csv_writer = csv.writer(self.log_file)
            
            # ヘッダー書き込み（フィルタ関連情報を追加）
            headers = [
                'timestamp', 'elapsed_time',
                'pos_x', 'pos_y', 'pos_z',
                'raw_pos_x', 'raw_pos_y', 'raw_pos_z',  # 生データ
                'error_x', 'error_y',
                'target_x', 'target_y',
                'roll_ref_rad', 'pitch_ref_rad',
                'roll_ref_deg', 'pitch_ref_deg',
                'pid_x_p', 'pid_x_i', 'pid_x_d',
                'pid_y_p', 'pid_y_i', 'pid_y_d',
                'frame_number', 'marker_count',
                'send_success', 'control_active',
                'loop_time_ms',
                'frame_dt_ms',
                'command_sequence', 'feedback_sequence',
                'feedback_roll_rad', 'feedback_pitch_rad',
                'feedback_latency_ms', 'feedback_age_ms',
                'feedback_match', 'feedback_delta_roll', 'feedback_delta_pitch',
                # フィルタ関連
                'is_outlier', 'used_prediction', 'confidence',
                'consecutive_outliers', 'data_valid',
                'data_source', 'filter_threshold', 'tracking_valid',
                'rb_error', 'rb_marker_count',
                'rb_pos_x', 'rb_pos_y', 'rb_pos_z',
                'rb_qx', 'rb_qy', 'rb_qz', 'rb_qw',
                'rb_roll_deg', 'rb_pitch_deg', 'rb_yaw_deg'
            ]
            self.csv_writer.writerow(headers)
            
            self.log_start_time = time.time()
            print(f"✓ ログファイル作成: {filename}")
            
        except Exception as e:
            print(f"✗ ログファイル作成エラー: {e}")
            self.log_file = None
            self.csv_writer = None
    
    def log_data(self, pos_x, pos_y, pos_z, error_x, error_y,
                  roll_ref, pitch_ref, send_success, loop_time,
                  target_position=None,
                  filter_result=None, is_data_valid=True,
                  confidence=1.0, data_source=None,
                  frame_dt=None,
                  command_sequence=None, feedback_info=None):
        """制御データをCSVに記録（拡張版）"""
        if not self.csv_writer:
            return

        try:
            # PID成分取得
            pid_components = self.pid_controller.get_all_components()

            # 現在時刻と経過時間
            current_time = time.time()
            elapsed = current_time - self.log_start_time if self.log_start_time else 0

            # フィルタ関連情報の取得
            threshold = None
            data_source_val = data_source if data_source is not None else self.last_data_source
            tracking_valid_flag = 1 if is_data_valid else 0
            rb_error = None
            confidence_val = confidence

            if filter_result:
                raw_x, raw_y, raw_z = filter_result['raw_position']
                is_outlier = filter_result['is_outlier']
                used_prediction = filter_result['used_prediction']
                confidence_val = filter_result.get('confidence', confidence_val)
                consecutive_outliers = filter_result['consecutive_outliers']
                threshold = filter_result.get('threshold', None)
                if filter_result.get('tracking_valid') is not None:
                    tracking_valid_flag = 1 if filter_result['tracking_valid'] else 0
                rb_error = filter_result.get('rigid_body_error', None)
                data_source_val = filter_result.get('source', data_source_val)
            else:
                raw_x, raw_y, raw_z = pos_x, pos_y, pos_z
                is_outlier = False
                used_prediction = False
                consecutive_outliers = 0
                threshold = None

            rb_pose = self.last_rigid_body_pose or {}
            rb_pos = rb_pose.get('position_drone')
            rb_quat = rb_pose.get('rotation')
            rb_euler_deg = rb_pose.get('euler_deg')
            rb_tracking = 1 if rb_pose.get('tracking_valid') else 0
            rb_marker_count = rb_pose.get('marker_count')
            rb_error_logged = rb_error if rb_error is not None else rb_pose.get('error')

            # フィードバック関連情報
            command_sequence_str = "" if command_sequence is None else str(command_sequence)
            feedback_sequence_str = ""
            feedback_roll_str = ""
            feedback_pitch_str = ""
            feedback_latency_ms_str = ""
            feedback_age_ms_str = ""
            feedback_match_str = ""
            feedback_delta_roll = ""
            feedback_delta_pitch = ""

            if feedback_info:
                fb_sequence = feedback_info.get('sequence')
                fb_roll = feedback_info.get('roll')
                fb_pitch = feedback_info.get('pitch')
                fb_latency = feedback_info.get('latency')
                fb_timestamp = feedback_info.get('timestamp')
                fb_matched = feedback_info.get('matched')

                if fb_sequence is not None:
                    feedback_sequence_str = str(fb_sequence)

                if fb_roll is not None:
                    feedback_roll_str = f"{fb_roll:.6f}"
                    feedback_delta_roll = f"{fb_roll - roll_ref:.6f}"

                if fb_pitch is not None:
                    feedback_pitch_str = f"{fb_pitch:.6f}"
                    feedback_delta_pitch = f"{fb_pitch - pitch_ref:.6f}"

                if fb_latency is not None:
                    feedback_latency_ms_str = f"{fb_latency * 1000:.3f}"

                if fb_timestamp is not None:
                    feedback_age = max(0.0, current_time - fb_timestamp)
                    feedback_age_ms_str = f"{feedback_age * 1000:.3f}"

                if fb_matched is not None:
                    feedback_match_str = "1" if fb_matched else "0"
                elif command_sequence is not None and fb_sequence is not None:
                    feedback_match_str = "1" if fb_sequence == command_sequence else "0"

            frame_dt_ms_str = ""
            if frame_dt is not None:
                frame_dt_ms_str = f"{frame_dt * 1000:.3f}"

            if target_position is None:
                target_position = self.last_target_position

            target_x = ""
            target_y = ""
            if target_position:
                try:
                    target_x = f"{float(target_position[0]):.6f}"
                    target_y = f"{float(target_position[1]):.6f}"
                except (TypeError, ValueError, IndexError):
                    target_x = ""
                    target_y = ""

            # データ行作成
            row = [
                datetime.now().isoformat(),  # timestamp
                f"{elapsed:.4f}",  # elapsed_time
                f"{pos_x:.6f}", f"{pos_y:.6f}", f"{pos_z:.6f}",  # filtered position
                f"{raw_x:.6f}", f"{raw_y:.6f}", f"{raw_z:.6f}",  # raw position
                f"{error_x:.6f}", f"{error_y:.6f}",  # errors
                target_x, target_y,
                f"{roll_ref:.6f}", f"{pitch_ref:.6f}",  # rad
                f"{roll_ref * 180 / 3.14159:.3f}", f"{pitch_ref * 180 / 3.14159:.3f}",  # deg
                f"{pid_components['x']['p']:.6f}", f"{pid_components['x']['i']:.6f}", f"{pid_components['x']['d']:.6f}",
                f"{pid_components['y']['p']:.6f}", f"{pid_components['y']['i']:.6f}", f"{pid_components['y']['d']:.6f}",
                self.current_frame_number,
                self.current_marker_count,
                1 if send_success else 0,
                1 if self.control_active else 0,
                f"{loop_time * 1000:.2f}",  # ms単位
                frame_dt_ms_str,
                command_sequence_str,
                feedback_sequence_str,
                feedback_roll_str,
                feedback_pitch_str,
                feedback_latency_ms_str,
                feedback_age_ms_str,
                feedback_match_str,
                feedback_delta_roll,
                feedback_delta_pitch,
                # フィルタ関連
                1 if is_outlier else 0,
                1 if used_prediction else 0,
                f"{confidence_val:.3f}",
                consecutive_outliers,
                1 if is_data_valid else 0,
                data_source_val or "",
                f"{threshold:.4f}" if threshold is not None else "",
                tracking_valid_flag,
                f"{rb_error_logged:.5f}" if rb_error_logged is not None else "",
                rb_marker_count if rb_marker_count is not None else "",
                "" if rb_pos is None else f"{rb_pos[0]:.6f}",
                "" if rb_pos is None else f"{rb_pos[1]:.6f}",
                "" if rb_pos is None else f"{rb_pos[2]:.6f}",
                "" if rb_quat is None else f"{rb_quat[0]:.6f}",
                "" if rb_quat is None else f"{rb_quat[1]:.6f}",
                "" if rb_quat is None else f"{rb_quat[2]:.6f}",
                "" if rb_quat is None else f"{rb_quat[3]:.6f}",
                "" if rb_euler_deg is None else f"{rb_euler_deg[0]:.3f}",
                "" if rb_euler_deg is None else f"{rb_euler_deg[1]:.3f}",
                "" if rb_euler_deg is None else f"{rb_euler_deg[2]:.3f}"
            ]
            
            self.csv_writer.writerow(row)
            
        except Exception as e:
            # ログエラーは制御に影響させない
            if self.packets_sent % 100 == 0:  # エラーを間引いて表示
                print(f"[Log Error] {e}")
    
    def connect_serial(self):
        """シリアルポートに接続"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=0.01,
                write_timeout=0.01
            )
            
            if self.ser.is_open:
                print(f"✓ シリアルポート {self.serial_port} に接続しました")
                time.sleep(2)  # ESP32の初期化待ち
                return True
                
        except serial.SerialException as e:
            print(f"✗ シリアル接続エラー: {e}")
            return False
            
    def disconnect_serial(self):
        """シリアルポートを切断"""
        if self.ser and self.ser.is_open:
            self.stop_serial_reader()
            self.ser.close()
            print("✓ シリアルポートを切断しました")
        self.ser = None
            
    def send_command(self, command):
        """
        文字列コマンドをESP32に送信（start/stop）
        """
        if not self.ser or not self.ser.is_open:
            return False
            
        try:
            message = command + '\n'
            self.ser.write(message.encode('utf-8'))
            print(f"→ コマンド送信: {command}")
            return True
        except Exception as e:
            print(f"✗ コマンド送信エラー: {e}")
            return False
            
    def send_angle_command(self, roll_ref, pitch_ref):
        """
        角度指令をESP32に送信
        Args:
            roll_ref: ロール角指令値 [rad]
            pitch_ref: ピッチ角指令値 [rad]
        """
        if not self.ser or not self.ser.is_open:
            return False, None
        
        # シーケンス番号を更新（32bitでロールオーバー）
        self.command_sequence = (self.command_sequence + 1) & 0xFFFFFFFF
        sequence_bytes = struct.pack('<I', self.command_sequence)
        angle_bytes = struct.pack('<ff', roll_ref, pitch_ref)
        payload = sequence_bytes + angle_bytes
        checksum = sum(payload) & 0xFF
        packet = ANGLE_HEADER + payload + bytes([checksum])

        send_time = time.time()
        success = False

        try:
            self.ser.write(packet)
            self.packets_sent += 1
            success = True
        except Exception as e:
            if self.packets_sent % 100 == 0:  # エラーを間引いて表示
                print(f"✗ 角度送信エラー: {e}")
        finally:
            command_info = {
                'sequence': self.command_sequence,
                'timestamp': send_time,
                'roll': roll_ref,
                'pitch': pitch_ref,
                'success': success
            }
            with self.feedback_lock:
                self.pending_commands[self.command_sequence] = command_info
                # 古い履歴を間引き
                while len(self.pending_commands) > 500:
                    oldest_seq = next(iter(self.pending_commands))
                    if oldest_seq == self.command_sequence:
                        break
                    self.pending_commands.pop(oldest_seq, None)

        return success, self.command_sequence

    def start_serial_reader(self):
        """リレーからのフィードバック受信スレッドを起動"""
        if not self.ser or not self.ser.is_open:
            return
        if self.feedback_thread and self.feedback_thread.is_alive():
            return

        with self.feedback_lock:
            self.pending_commands.clear()
            self.latest_feedback = None
            self.last_feedback_sequence = None

        self.serial_buffer = bytearray()
        self.serial_text_buffer = bytearray()
        self.feedback_stop_event.clear()

        self.feedback_thread = threading.Thread(
            target=self.serial_reader_loop,
            name="SerialFeedbackReader",
            daemon=True
        )
        self.feedback_thread.start()

    def stop_serial_reader(self):
        """フィードバック受信スレッドを停止"""
        self.feedback_stop_event.set()
        if self.feedback_thread and self.feedback_thread.is_alive():
            self.feedback_thread.join(timeout=1.0)
        self.feedback_thread = None
        self.serial_buffer = bytearray()
        self.serial_text_buffer = bytearray()

    def serial_reader_loop(self):
        """リレーからのフィードバックを非同期に読み取る"""
        header_byte = FEEDBACK_HEADER[0]
        frame_size = FEEDBACK_FRAME_SIZE
        payload_len = frame_size - 2

        while not self.feedback_stop_event.is_set():
            if not self.ser or not self.ser.is_open:
                time.sleep(0.01)
                continue

            try:
                data = self.ser.read(64)
            except (serial.SerialException, OSError) as e:
                print(f"[Serial Read Error] {e}")
                self.feedback_stop_event.set()
                break

            if data:
                self.serial_buffer.extend(data)
                self._process_serial_buffer(frame_size, payload_len, header_byte)
            else:
                time.sleep(0.002)

        # 残りのテキストを出力
        if self.serial_text_buffer:
            residual = self.serial_text_buffer.replace(b'\r', b'').decode('utf-8', errors='ignore').strip()
            if residual:
                print(f"[Relay] {residual}")
            self.serial_text_buffer.clear()

    def _process_serial_buffer(self, frame_size, payload_len, header_byte):
        """シリアルバッファからバイナリフレームとテキストを振り分け"""
        header_byte_array = bytes([header_byte])

        while True:
            header_index = self.serial_buffer.find(header_byte_array)
            if header_index == -1:
                if self.serial_buffer:
                    remainder = bytes(self.serial_buffer)
                    self.serial_buffer.clear()
                    self._append_serial_text(remainder)
                break

            if header_index > 0:
                chunk = self.serial_buffer[:header_index]
                del self.serial_buffer[:header_index]
                self._append_serial_text(chunk)

            if len(self.serial_buffer) < frame_size:
                break

            payload = self.serial_buffer[1:1 + payload_len]
            checksum = self.serial_buffer[frame_size - 1]
            if (sum(payload) & 0xFF) == checksum:
                sequence, roll, pitch = struct.unpack('<Iff', payload)
                self.handle_feedback_frame(sequence, roll, pitch)
                del self.serial_buffer[:frame_size]
            else:
                # 破損フレームはテキスト扱いとして破棄
                self.feedback_stats['invalid'] += 1
                corrupted = self.serial_buffer[:1]
                del self.serial_buffer[:1]
                self._append_serial_text(corrupted)

    def _append_serial_text(self, data):
        """シリアルログを逐次デコードして表示"""
        if not data:
            return

        if isinstance(data, (bytes, bytearray)):
            self.serial_text_buffer.extend(data)
        else:
            self.serial_text_buffer.extend(bytes(data))

        while True:
            newline_pos = self.serial_text_buffer.find(b'\n')
            if newline_pos == -1:
                break
            line = self.serial_text_buffer[:newline_pos + 1]
            del self.serial_text_buffer[:newline_pos + 1]
            text = line.replace(b'\r', b'').decode('utf-8', errors='ignore').strip()
            if text:
                print(f"[Relay] {text}")

        if len(self.serial_text_buffer) > 512:
            overflow = self.serial_text_buffer.replace(b'\r', b'').decode('utf-8', errors='ignore').strip()
            if overflow:
                print(f"[Relay] {overflow}")
            self.serial_text_buffer.clear()

    def handle_feedback_frame(self, sequence, roll, pitch):
        """受信したフィードバックフレームを処理"""
        now = time.time()
        with self.feedback_lock:
            command_info = self.pending_commands.pop(sequence, None)
            latency = None
            matched = False
            if command_info:
                latency = max(0.0, now - command_info['timestamp'])
                matched = True

            self.latest_feedback = {
                'sequence': sequence,
                'roll': roll,
                'pitch': pitch,
                'timestamp': now,
                'latency': latency,
                'matched': matched
            }

            if command_info:
                self.latest_feedback['command_roll'] = command_info.get('roll')
                self.latest_feedback['command_pitch'] = command_info.get('pitch')

            self.last_feedback_sequence = sequence
            self.feedback_stats['received'] += 1

    def get_feedback_snapshot(self):
        """最新のフィードバック情報をコピーして返す"""
        with self.feedback_lock:
            if not self.latest_feedback:
                return None
            return dict(self.latest_feedback)

    def update_control_from_frame(self, filtered_position, filter_result,
                                  frame_time_monotonic, frame_dt):
        """新規フレーム到着時にPID更新して最新指令を保持"""
        if not self.control_active:
            return

        pos_x, pos_y, _ = filtered_position
        target_x, target_y = self.get_target_position(frame_time_monotonic)
        error_x = target_x - pos_x
        error_y = target_y - pos_y

        confidence = 0.0
        is_data_valid = False
        consecutive_outliers = 0

        if filter_result:
            confidence = filter_result.get('confidence', 1.0)
            consecutive_outliers = filter_result.get('consecutive_outliers', 0)
            is_data_valid = (
                confidence > 0.1 and
                not filter_result.get('is_outlier', False) and
                filter_result.get('tracking_valid', True)
            )

        if frame_dt is not None and frame_dt > self.frame_hold_timeout:
            is_data_valid = False
            self.pid_controller.set_anomaly_state(True)

        if consecutive_outliers > 3 or confidence < 0.3:
            self.pid_controller.set_anomaly_state(True)
        elif consecutive_outliers == 0 and confidence > 0.5:
            self.pid_controller.set_anomaly_state(False)

        roll_ref, pitch_ref = self.pid_controller.calculate(
            error_x, error_y, frame_time_monotonic, is_data_valid
        )

        if confidence < self.confidence_zero_threshold:
            roll_ref = 0.0
            pitch_ref = 0.0
        elif confidence < self.confidence_scale_threshold:
            roll_ref *= confidence
            pitch_ref *= confidence

        with self.position_lock:
            self.last_roll_ref = roll_ref
            self.last_pitch_ref = pitch_ref
            self.last_error_x = error_x
            self.last_error_y = error_y
            self.last_target_position = (target_x, target_y)
            self.last_control_confidence = confidence
            self.last_control_data_valid = is_data_valid
            
    def receive_mocap_frame(self, data_dict):
        """
        NatNetからのコールバック（リジッドボディ + マーカーデータを統合）
        """
        mocap_data = data_dict.get("mocap_data", None)
        if mocap_data is None:
            return

        frame_num = data_dict.get("frame_number", 0)
        frame_time_monotonic = time.monotonic()
        frame_time_wall = time.time()
        prev_frame_time = self.last_frame_time_monotonic
        frame_dt = None if prev_frame_time is None else (frame_time_monotonic - prev_frame_time)
        self.filter_stats['total_frames'] += 1

        # リジッドボディ姿勢を取得
        rigid_pose = self.extract_rigid_body_pose(mocap_data)
        self.last_rigid_body_pose = rigid_pose

        # ラベル付きマーカー中心を計算（フォールバック用）
        marker_position = None
        marker_count = 0
        if mocap_data.labeled_marker_data:
            markers = mocap_data.labeled_marker_data.labeled_marker_list
            marker_count = len(markers)
            if marker_count > 0:
                marker_position = self.calculate_center_position(markers)

        # データソースを選択（リジッドボディ優先、無効ならマーカーにフォールバック）
        selected_position = None
        selected_source = "markers"
        tracking_valid = False
        quality_weight = None
        rigid_body_error = None
        effective_marker_count = marker_count

        if rigid_pose and rigid_pose['tracking_valid']:
            selected_position = rigid_pose['position_drone']
            selected_source = "rigid_body"
            tracking_valid = True
            quality_weight = rigid_pose['quality']
            rigid_body_error = rigid_pose['error']
            if rigid_pose['marker_count'] > 0:
                effective_marker_count = rigid_pose['marker_count']
        elif marker_position is not None:
            selected_position = marker_position
            selected_source = "markers"
            tracking_valid = marker_count >= 3
        elif rigid_pose:
            # リジッドボディは存在するがtracking_invalid。最後の姿勢を低信頼で使用
            selected_position = rigid_pose['position_drone']
            selected_source = "rigid_body"
            tracking_valid = False
            quality_weight = rigid_pose['quality']
            rigid_body_error = rigid_pose['error']
            if rigid_pose['marker_count'] > 0:
                effective_marker_count = rigid_pose['marker_count']

        if selected_position is None:
            # データがない場合は記録のみ更新し終了
            with self.position_lock:
                self.last_filter_result = None
                self.last_data_source = "none"
                self.last_frame_time_monotonic = frame_time_monotonic
                self.last_frame_wall_time = frame_time_wall
                self.last_frame_dt = frame_dt
                self.current_frame_number = frame_num
                self.current_marker_count = effective_marker_count
                self.last_control_confidence = 0.0
                self.last_control_data_valid = False
            return

        filter_result = self.position_filter.process_position(
            selected_position,
            marker_count=effective_marker_count,
            current_time=frame_time_monotonic,
            tracking_valid=tracking_valid,
            quality_weight=quality_weight,
            rigid_body_error=rigid_body_error,
            source=selected_source
        )

        filtered_position = filter_result['filtered_position']

        with self.position_lock:
            self.current_position = filtered_position
            self.last_valid_position = filtered_position
            self.last_filter_result = filter_result
            self.current_frame_number = frame_num
            self.current_marker_count = effective_marker_count
            self.last_data_source = selected_source
            self.last_frame_time_monotonic = frame_time_monotonic
            self.last_frame_wall_time = frame_time_wall
            self.last_frame_dt = frame_dt

        if filter_result['is_outlier']:
            self.filter_stats['outliers_detected'] += 1
        if filter_result['used_prediction']:
            self.filter_stats['predictions_used'] += 1

        self.update_control_from_frame(
            filtered_position,
            filter_result,
            frame_time_monotonic,
            frame_dt
        )

        if frame_num % 10 == 0:
            source_label = "RB" if selected_source == "rigid_body" else "Markers"
            print(f"[NatNet] Frame:{frame_num} Src:{source_label} Markers:{effective_marker_count} "
                  f"Pos=({filtered_position[0]:+.3f}, {filtered_position[1]:+.3f}, {filtered_position[2]:+.3f}) m "
                  f"Conf={filter_result['confidence']:.2f}")
            if filter_result['is_outlier']:
                raw_pos = filter_result['raw_position']
                print(f"  ⚠ 異常値検出（生データ: {raw_pos[0]:.3f}, {raw_pos[1]:.3f}）")
                        
    def convert_motive_to_drone(self, position):
        """
        Motive座標系(X,Y,Z)をドローン座標系(Z,X,Y)へ変換
        """
        if position is None:
            return None
        motive_x, motive_y, motive_z = position
        src = (motive_x, motive_y, motive_z)
        x_map = self.axis_map.get("x", (2, 1))
        y_map = self.axis_map.get("y", (0, 1))
        z_map = self.axis_map.get("z", (1, 1))
        drone_x = src[x_map[0]] * x_map[1]
        drone_y = src[y_map[0]] * y_map[1]
        drone_z = src[z_map[0]] * z_map[1]
        return (drone_x, drone_y, drone_z)

    @staticmethod
    def quaternion_to_euler_xyz(qx, qy, qz, qw):
        """四元数(x,y,z,w)をロール・ピッチ・ヨー(rad)へ変換"""
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = copysign(pi / 2.0, sinp)
        else:
            pitch = asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = atan2(siny_cosp, cosy_cosp)
        return (roll, pitch, yaw)

    def extract_rigid_body_pose(self, mocap_data):
        """
        指定IDのリジッドボディ姿勢を取得し、ドローン座標に変換
        """
        rigid_body_data = getattr(mocap_data, 'rigid_body_data', None)
        if rigid_body_data is None:
            return None

        rigid_bodies = getattr(rigid_body_data, 'rigid_body_list', None)
        if not rigid_bodies:
            return None

        for rigid_body in rigid_bodies:
            if getattr(rigid_body, 'id_num', None) != self.rigid_body_id:
                continue

            motive_pos = tuple(rigid_body.pos)
            drone_pos = self.convert_motive_to_drone(motive_pos)
            rotation = tuple(rigid_body.rot)
            tracking_valid = getattr(rigid_body, 'tracking_valid', False)
            error = getattr(rigid_body, 'error', None)
            marker_count = len(getattr(rigid_body, 'rb_marker_list', []))

            euler_rad = self.quaternion_to_euler_xyz(*rotation)
            euler_deg = tuple(degrees(val) for val in euler_rad)

            quality = 1.0
            if error is not None:
                quality = 1.0 / (1.0 + max(0.0, error))
            if not tracking_valid:
                quality *= 0.5
            quality = max(0.05, min(1.0, quality))

            return {
                'timestamp': time.time(),
                'position_motive': motive_pos,
                'position_drone': drone_pos,
                'rotation': rotation,
                'euler_rad': euler_rad,
                'euler_deg': euler_deg,
                'tracking_valid': tracking_valid,
                'error': error,
                'marker_count': marker_count,
                'quality': quality
            }

        return None

    def calculate_center_position(self, markers):
        """
        マーカーリストから中心点を計算
        座標系変換: Motive(X,Y,Z) → ドローン(Z,X,Y)
        """
        if not markers or len(markers) == 0:
            return None
            
        # Motiveの座標系での平均を計算
        sum_x = sum(marker.pos[0] for marker in markers)
        sum_y = sum(marker.pos[1] for marker in markers)
        sum_z = sum(marker.pos[2] for marker in markers)
        
        num_markers = len(markers)
        center_x = sum_x / num_markers
        center_y = sum_y / num_markers
        center_z = sum_z / num_markers
        
        return self.convert_motive_to_drone((center_x, center_y, center_z))
        
    def control_loop(self):
        """
        制御ループ（別スレッドで実行）
        拡張版：フィルタ結果を考慮したPID制御
        """
        print("制御ループ開始（拡張版）")
        control_rate = self.control_rate_hz  # Hz
        period = self.control_period
        no_data_counter = 0

        while not self.shutdown_flag:
            loop_start = time.monotonic()

            if self.control_active:
                with self.position_lock:
                    position_snapshot = self.current_position
                    filter_result = getattr(self, 'last_filter_result', None)
                    data_source = self.last_data_source
                    roll_ref = self.last_roll_ref
                    pitch_ref = self.last_pitch_ref
                    error_x = self.last_error_x
                    error_y = self.last_error_y
                    target_position = self.last_target_position
                    confidence = self.last_control_confidence
                    is_data_valid = self.last_control_data_valid
                    frame_dt = self.last_frame_dt
                    frame_time = self.last_frame_time_monotonic

                if position_snapshot:
                    pos_x, pos_y, pos_z = position_snapshot
                    now = time.monotonic()
                    frame_age = None if frame_time is None else (now - frame_time)
                    frame_stale = frame_age is None or frame_age > self.frame_hold_timeout
                    if frame_stale:
                        is_data_valid = False

                    # 座標系の確認用: 符号を反転する必要がある可能性
                    # 現在: X誤差→ロール、Y誤差→ピッチ
                    # もし逆の場合は以下のコメントを外す
                    # pitch_ref, roll_ref = roll_ref, pitch_ref
                    
                    # 角度指令送信
                    success, sequence = self.send_angle_command(roll_ref, pitch_ref)
                    self.last_command_sequence = sequence
                    feedback_snapshot = self.get_feedback_snapshot()

                    # データをログに記録（100Hz全て、拡張版）
                    loop_time = time.monotonic() - loop_start
                    self.log_data(pos_x, pos_y, pos_z, error_x, error_y,
                                  roll_ref, pitch_ref, success, loop_time,
                                  target_position,
                                  filter_result, is_data_valid,
                                  confidence, data_source,
                                  frame_dt=frame_dt,
                                  command_sequence=sequence,
                                  feedback_info=feedback_snapshot)
                    
                    # デバッグ出力（5Hzで継続的に表示）
                    if self.packets_sent % 20 == 0:
                        roll_deg = roll_ref * 180 / 3.14159
                        pitch_deg = pitch_ref * 180 / 3.14159
                        feedback_seq = feedback_snapshot.get('sequence') if feedback_snapshot else None
                        feedback_latency = feedback_snapshot.get('latency') if feedback_snapshot else None
                        feedback_match = bool(feedback_snapshot and feedback_seq == sequence)
                        latency_text = f"{feedback_latency * 1000:.1f}ms" if feedback_latency is not None else "-"
                        ack_text = feedback_seq if feedback_seq is not None else "-"
                        ack_status = "✓" if feedback_match else "…"

                        print(f"[Control] Src:{data_source} 位置: ({pos_x:+6.3f}, {pos_y:+6.3f}) m | "
                              f"誤差: ({error_x:+6.3f}, {error_y:+6.3f}) m | "
                              f"指令: R={roll_deg:+6.1f}°, P={pitch_deg:+6.1f}° | "
                              f"Conf:{confidence:.2f} 送信: {'OK' if success else 'FAIL'} | "
                              f"ACK:{ack_text} {ack_status} ({latency_text})")
                    
                    no_data_counter = 0
                else:
                    # 位置データがない場合
                    no_data_counter += 1
                    if no_data_counter % 100 == 0:
                        print("[Warning] No position data available!")

                    feedback_snapshot = self.get_feedback_snapshot()
                    success = False
                    sequence = self.last_command_sequence

                    # データなしでもログに記録（NaNまたは0として）
                    loop_time = time.monotonic() - loop_start
                    self.log_data(0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, success, loop_time,
                                  target_position,
                                  None, False, confidence=0.0, data_source="none",
                                  frame_dt=None,
                                  command_sequence=sequence,
                                  feedback_info=feedback_snapshot)
                    
            # 制御周期を維持
            elapsed = time.monotonic() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)
                
        print("制御ループ終了")
        
    def start_hovering(self):
        """ホバリング開始"""
        if self.is_flying:
            print("既に飛行中です")
            return
            
        print("\n=== ホバリング開始シーケンス ===")
        
        # 先に制御を有効化（ただし離陸前は角度0を維持）
        self.control_active = False  # 最初はFalseで開始
        
        # 1. NatNet接続
        print("1. Optitrack/Motiveに接続中...")
        self.natnet_client = NatNetClient()
        self.natnet_client.set_server_address("127.0.0.1")
        self.natnet_client.set_client_address("127.0.0.1")
        self.natnet_client.set_use_multicast(True)
        
        # コールバック登録（drone_tracker.pyと同じ）
        self.natnet_client.new_frame_with_data_listener = self.receive_mocap_frame
        self.natnet_client.set_print_level(0)  # デバッグ出力OFF
        
        # NatNet開始（drone_tracker.pyと同じ'd'引数）
        if not self.natnet_client.run('d'):
            print("   ✗ 接続に失敗しました")
            return
        
        time.sleep(1)
        
        if not self.natnet_client.connected():
            print("   ✗ Motiveが見つかりません")
            print("   Motiveのストリーミング設定を確認してください:")
            print("     1. Edit → Settings → Streaming を開く")
            print("     2. Broadcast Frame Data にチェック")
            print("     3. Local Interface でネットワークインターフェースを選択")
            self.natnet_client.shutdown()
            return
        
        # データ記述を要求
        self.natnet_client.send_request(
            self.natnet_client.command_socket,
            self.natnet_client.NAT_REQUEST_MODELDEF,
            "",
            ("127.0.0.1", self.natnet_client.command_port)
        )
        
        print("   ✓ Optitrack接続完了")
        
        # 2. 位置データ確認
        print("2. 位置データの確認...")
        wait_start = time.time()
        max_wait_seconds = 10.0
        last_notice_second = -1
        valid_position = None
        valid_filter = None

        while time.time() - wait_start < max_wait_seconds:
            with self.position_lock:
                snapshot_position = self.current_position
                snapshot_filter = self.last_filter_result

            if snapshot_position and snapshot_filter:
                confidence = snapshot_filter.get('confidence', 0.0)
                tracking_valid = snapshot_filter.get('tracking_valid', True)
                is_outlier = snapshot_filter.get('is_outlier', False)
                if confidence >= self.confidence_zero_threshold and tracking_valid and not is_outlier:
                    valid_position = snapshot_position
                    valid_filter = snapshot_filter
                    break

            elapsed = time.time() - wait_start
            current_second = int(elapsed)
            if current_second != last_notice_second:
                last_notice_second = current_second
                print(f"   位置データ待機中... {elapsed:.1f}秒")
            time.sleep(0.1)

        if not valid_position:
            print("   ✗ 有効な位置データが取得できませんでした。Motiveのリジッドボディ/マーカー状態を確認し、再度 start を実行してください。")
            if self.natnet_client:
                self.natnet_client.shutdown()
                self.natnet_client = None
            return

        confidence = valid_filter.get('confidence', 0.0) if valid_filter else 0.0
        print(f"   ✓ 初期位置取得: ({valid_position[0]:.3f}, {valid_position[1]:.3f}, {valid_position[2]:.3f}) m (Conf={confidence:.2f})")

        # 3. データロギング開始
        print("3. データロギングを開始...")
        self.start_logging()

        # 4. フィードバック受信を準備...
        print("4. フィードバック受信を準備...")
        if self.ser and self.ser.is_open:
            try:
                self.ser.reset_input_buffer()
            except (serial.SerialException, OSError):
                pass
        self.start_serial_reader()
        print("   ✓ フィードバック受信準備完了")

        # 5. 制御ループを開始
        print("5. 制御ループを開始...")
        self.shutdown_flag = False
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        print("   ✓ 制御ループ開始")

        # 6. 離陸コマンド送信
        print("6. 離陸コマンドを送信...")
        if self.send_command("start"):
            print("   ✓ 離陸コマンド送信完了")
            
            # 7. 即座に制御開始（離陸と同時に制御）
            print("7. 位置制御を開始...")
            self.pid_controller.reset()
            time.sleep(0.5)  # 短い待機
            self.control_active = True  # 制御開始
            self.is_flying = True
            self.on_control_start()
            print("   ✓ ホバリング制御開始")
            
            if self.current_position:
                print(f"   {self.get_target_description()}")
                print(f"   現在位置: ({self.current_position[0]:.3f}, {self.current_position[1]:.3f}, {self.current_position[2]:.3f}) m")
            
            print(f"\n{self.get_active_mode_message()}")
        else:
            print("   ✗ 離陸コマンド送信失敗")
            self.stop_hovering()
            
    def stop_hovering(self):
        """ホバリング停止"""
        if not self.is_flying:
            print("飛行していません")
            return
            
        print("\n=== 着陸シーケンス ===")
        
        # 1. 制御停止
        print("1. 位置制御を停止...")
        self.control_active = False
        time.sleep(0.5)
        
        # 2. 着陸コマンド
        print("2. 着陸コマンドを送信...")
        if self.send_command("stop"):
            print("   ✓ 着陸コマンド送信完了")
            
        # 3. クリーンアップ
        print("3. クリーンアップ中...")
        self.shutdown_flag = True
        
        if self.control_thread:
            self.control_thread.join(timeout=2)
            self.control_thread = None

        self.stop_serial_reader()
        if self.ser and self.ser.is_open:
            try:
                self.ser.reset_input_buffer()
            except (serial.SerialException, OSError):
                pass
            
        if self.natnet_client:
            self.natnet_client.shutdown()
            self.natnet_client = None
            
        # 4. ログファイルをクローズ
        print("4. データロギングを停止...")
        self.close_log_file()
        
        self.is_flying = False
        print("   ✓ 着陸完了")
        
    def run(self):
        """メインループ"""
        print("\n" + "="*50)
        print("Optitrack 原点ホバリング制御システム")
        print("="*50)
        print("コマンド:")
        print("  start - 離陸してホバリング開始")
        print("  stop  - 着陸")
        print("  exit  - プログラム終了")
        print("="*50 + "\n")
        
        # シリアルポート接続
        if not self.connect_serial():
            print("ESP32への接続に失敗しました")
            return
            
        try:
            while True:
                command = input("\nコマンド > ").strip().lower()
                
                if command == "exit":
                    if self.is_flying:
                        print("着陸してから終了します...")
                        self.stop_hovering()
                    print("プログラムを終了します")
                    break
                    
                elif command == "start":
                    self.start_hovering()
                    
                elif command == "stop":
                    self.stop_hovering()
                    
                elif command == "":
                    continue
                    
                else:
                    print(f"✗ 不明なコマンド: {command}")
                    
        except KeyboardInterrupt:
            print("\n\nCtrl+Cが押されました")
            
        finally:
            # 安全な終了処理
            if self.is_flying:
                print("安全のため着陸します...")
                self.stop_hovering()
                time.sleep(2)
                
            self.disconnect_serial()
            print("\nプログラムを終了しました")

def main():
    """メイン関数"""
    controller = HoveringController()
    controller.run()

if __name__ == "__main__":
    main()
