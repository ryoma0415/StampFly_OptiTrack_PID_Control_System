#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
位置データフィルタリングモジュール
マーカー検出の不安定性に対処するための位置データ前処理
"""

import numpy as np
from collections import deque
import time


class PositionFilter:
    """
    位置データの異常検出とフィルタリング
    - 位置の急変を検出
    - 移動平均による平滑化
    - 異常時の位置予測/補間
    """

    def __init__(self,
                 window_size=5,
                 outlier_threshold=0.1,
                 velocity_window=3,
                 enable_prediction=True,
                 max_outlier_threshold=0.4):
        """
        初期化

        Args:
            window_size: 移動平均のウィンドウサイズ
            outlier_threshold: 異常値判定の閾値 [m]
            velocity_window: 速度推定に使用するフレーム数
            enable_prediction: 位置予測の有効/無効
        """
        # フィルタパラメータ
        self.window_size = window_size
        self.base_threshold = outlier_threshold
        self.max_threshold = max(outlier_threshold, max_outlier_threshold)
        self.min_threshold = max(0.02, outlier_threshold * 0.5)
        self.dynamic_threshold = outlier_threshold
        self.velocity_window = velocity_window
        self.enable_prediction = enable_prediction

        # 位置履歴（移動平均用）
        self.position_history = deque(maxlen=window_size)

        # 速度推定用履歴
        self.velocity_history = deque(maxlen=velocity_window)
        self.time_history = deque(maxlen=velocity_window)

        # 最後の有効な位置
        self.last_valid_position = None
        self.last_valid_time = None
        self.prev_valid_position = None

        # 推定速度
        self.estimated_velocity = np.array([0.0, 0.0, 0.0])
        self.velocity_magnitude_ema = 0.0
        self.step_distance_ema = 0.0
        self.dt_ema = None

        # 異常検出フラグ
        self.outlier_detected = False
        self.outlier_count = 0
        self.consecutive_outliers = 0

        # 統計情報
        self.total_samples = 0
        self.outlier_samples = 0
        self.last_source = None
        self.last_confidence = 1.0

    def reset(self):
        """フィルタの内部状態をリセット"""
        self.position_history.clear()
        self.velocity_history.clear()
        self.time_history.clear()
        self.last_valid_position = None
        self.last_valid_time = None
        self.prev_valid_position = None
        self.estimated_velocity = np.array([0.0, 0.0, 0.0])
        self.velocity_magnitude_ema = 0.0
        self.step_distance_ema = 0.0
        self.dynamic_threshold = self.base_threshold
        self.dt_ema = None
        self.outlier_detected = False
        self.outlier_count = 0
        self.consecutive_outliers = 0
        self.last_source = None
        self.last_confidence = 1.0

    def _compute_dynamic_threshold(self, distance, time_diff, marker_count,
                                   tracking_valid, quality_weight):
        """
        直近の動きやトラッキング品質に基づいて閾値を計算
        """
        motion_allowance = self.step_distance_ema * 2.5

        if time_diff is None and self.dt_ema is not None:
            time_diff = self.dt_ema

        speed_allowance = 0.0
        if time_diff is not None:
            speed_allowance = self.velocity_magnitude_ema * max(time_diff, 0.0) * 1.5

        adaptive = self.base_threshold + motion_allowance + speed_allowance

        if marker_count is not None:
            marker_ratio = min(max(marker_count, 0), 4) / 4.0
            adaptive *= (0.9 + 0.2 * marker_ratio)

        if not tracking_valid:
            adaptive *= 0.8

        if quality_weight is not None:
            adaptive *= (0.8 + 0.4 * max(0.0, min(quality_weight, 1.2)))

        adaptive = max(self.min_threshold, min(self.max_threshold, adaptive))

        if self.dynamic_threshold is None:
            self.dynamic_threshold = adaptive
        else:
            self.dynamic_threshold = 0.6 * self.dynamic_threshold + 0.4 * adaptive

        return self.dynamic_threshold

    def is_outlier(self, position, current_time=None, tracking_valid=True,
                   marker_count=None, quality_weight=None):
        """
        位置データが異常値かどうかを判定

        Args:
            position: (x, y, z) の位置タプル
            current_time: 現在時刻
            tracking_valid: トラッキングの信頼度フラグ
            marker_count: マーカー数
            quality_weight: リジッドボディなどからの品質指標 (0-1)

        Returns:
            tuple: (is_outlier, threshold)
        """
        if self.last_valid_position is None:
            return False, self.base_threshold

        # 前回の有効位置からの距離を計算
        pos_array = np.array(position)
        last_array = np.array(self.last_valid_position)
        distance = np.linalg.norm(pos_array - last_array)

        time_diff = None
        if current_time is not None and self.last_valid_time is not None:
            time_diff = current_time - self.last_valid_time
        elif self.last_valid_time is not None:
            time_diff = time.time() - self.last_valid_time

        threshold = self._compute_dynamic_threshold(
            distance, time_diff, marker_count, tracking_valid, quality_weight
        )

        return distance > threshold, threshold

    def estimate_velocity(self):
        """
        過去の位置履歴から速度を推定

        Returns:
            numpy.array: 推定速度ベクトル [vx, vy, vz]
        """
        if len(self.velocity_history) < 2:
            return np.array([0.0, 0.0, 0.0])

        # 最小二乗法による速度推定
        positions = np.array(list(self.velocity_history))
        times = np.array(list(self.time_history))

        if len(times) < 2:
            return self.estimated_velocity

        # 時刻を相対時間に変換
        rel_times = times - times[0]

        # 各軸で線形回帰
        velocity = np.zeros(3)
        for i in range(3):
            if rel_times[-1] > 0:
                # 簡易的な速度計算（最初と最後の点から）
                velocity[i] = (positions[-1, i] - positions[0, i]) / rel_times[-1]

        return velocity

    def predict_position(self, time_delta):
        """
        現在の速度推定値から位置を予測

        Args:
            time_delta: 予測時間差 [秒]

        Returns:
            numpy.array: 予測位置
        """
        if self.last_valid_position is None:
            return None

        predicted = np.array(self.last_valid_position) + self.estimated_velocity * time_delta
        return tuple(predicted)

    def apply_moving_average(self):
        """
        移動平均を適用

        Returns:
            tuple: 平滑化された位置 (x, y, z)
        """
        if len(self.position_history) == 0:
            return None

        positions = np.array(list(self.position_history))

        # 重み付き移動平均（新しいデータほど重みを大きく）
        weights = np.exp(np.linspace(-1, 0, len(positions)))
        weights /= weights.sum()

        averaged = np.average(positions, weights=weights, axis=0)
        return tuple(averaged)

    def process_position(self, position, marker_count=None, current_time=None,
                         tracking_valid=True, quality_weight=None,
                         rigid_body_error=None, source="markers"):
        """
        位置データを処理（メインのフィルタリング関数）

        Args:
            position: (x, y, z) の位置タプル
            marker_count: 検出されたマーカー数（オプション）
            current_time: 現在時刻（Noneの場合は自動取得）

        Returns:
            dict: {
                'filtered_position': フィルタ後の位置,
                'raw_position': 生の位置,
                'is_outlier': 異常値フラグ,
                'used_prediction': 予測使用フラグ,
                'confidence': 信頼度 (0-1)
            }
        """
        if current_time is None:
            current_time = time.time()

        self.total_samples += 1
        self.last_source = source

        # 初期化
        if self.last_valid_position is None:
            self.last_valid_position = position
            self.last_valid_time = current_time
            self.position_history.append(position)
            self.velocity_history.append(position)
            self.time_history.append(current_time)
            self.dynamic_threshold = self.base_threshold
            self.last_confidence = 1.0
            return {
                'filtered_position': position,
                'raw_position': position,
                'is_outlier': False,
                'used_prediction': False,
                'confidence': 1.0,
                'marker_count': marker_count,
                'consecutive_outliers': 0,
                'source': source,
                'threshold': self.dynamic_threshold,
                'tracking_valid': tracking_valid,
                'rigid_body_error': rigid_body_error
            }

        # 異常値判定
        is_outlier, threshold = self.is_outlier(
            position,
            current_time=current_time,
            tracking_valid=tracking_valid,
            marker_count=marker_count,
            quality_weight=quality_weight
        )

        if is_outlier:
            self.outlier_detected = True
            self.outlier_count += 1
            self.outlier_samples += 1
            self.consecutive_outliers += 1

            # 予測位置を使用
            if self.enable_prediction and self.last_valid_time is not None:
                time_delta = current_time - self.last_valid_time
                predicted_pos = self.predict_position(time_delta)

                if predicted_pos is not None:
                    filtered_position = predicted_pos
                    used_prediction = True
                else:
                    filtered_position = self.last_valid_position
                    used_prediction = False
            else:
                filtered_position = self.last_valid_position
                used_prediction = False

            confidence = 0.35
            if marker_count is not None:
                confidence *= max(0.25, marker_count / 4.0)
            if not tracking_valid:
                confidence *= 0.4
            if quality_weight is not None:
                confidence *= 0.5 + 0.5 * max(0.0, min(quality_weight, 1.0))
            if rigid_body_error is not None:
                confidence *= 1.0 / (1.0 + max(0.0, rigid_body_error))

        else:
            # 正常データ
            self.outlier_detected = False
            self.consecutive_outliers = 0

            # 履歴更新
            self.position_history.append(position)
            self.velocity_history.append(position)
            self.time_history.append(current_time)

            # 速度推定を更新
            self.estimated_velocity = self.estimate_velocity()
            velocity_mag = np.linalg.norm(self.estimated_velocity)
            self.velocity_magnitude_ema = (
                0.7 * self.velocity_magnitude_ema + 0.3 * velocity_mag
            )

            # 移動平均を適用
            filtered_position = self.apply_moving_average()
            if filtered_position is None:
                filtered_position = position

            # 有効位置を更新
            if self.last_valid_position is not None:
                step_distance = np.linalg.norm(
                    np.array(filtered_position) - np.array(self.last_valid_position)
                )
                self.step_distance_ema = (
                    0.7 * self.step_distance_ema + 0.3 * step_distance
                )
                self.prev_valid_position = self.last_valid_position

            self.last_valid_position = filtered_position
            self.last_valid_time = current_time

            if len(self.time_history) >= 2:
                dt = self.time_history[-1] - self.time_history[-2]
                if dt > 0:
                    self.dt_ema = (
                        0.8 * self.dt_ema + 0.2 * dt
                        if self.dt_ema is not None else dt
                    )

            used_prediction = False

            confidence = 1.0
            if marker_count is not None:
                marker_ratio = min(max(marker_count, 0), 4) / 4.0
                confidence *= 0.7 + 0.3 * marker_ratio
            if not tracking_valid:
                confidence *= 0.7
            if quality_weight is not None:
                confidence *= max(0.4, min(1.0, quality_weight))
            if rigid_body_error is not None:
                confidence *= 1.0 / (1.0 + max(0.0, rigid_body_error))

        confidence = max(0.05, min(1.0, confidence))
        self.last_confidence = confidence

        return {
            'filtered_position': filtered_position,
            'raw_position': position,
            'is_outlier': is_outlier,
            'used_prediction': used_prediction,
            'confidence': confidence,
            'marker_count': marker_count,
            'consecutive_outliers': self.consecutive_outliers,
            'source': source,
            'threshold': threshold,
            'tracking_valid': tracking_valid,
            'rigid_body_error': rigid_body_error
        }

    def get_statistics(self):
        """
        フィルタの統計情報を取得

        Returns:
            dict: 統計情報
        """
        outlier_rate = 0.0
        if self.total_samples > 0:
            outlier_rate = self.outlier_samples / self.total_samples

        return {
            'total_samples': self.total_samples,
            'outlier_samples': self.outlier_samples,
            'outlier_rate': outlier_rate,
            'current_threshold': self.dynamic_threshold,
            'consecutive_outliers': self.consecutive_outliers,
            'estimated_velocity': self.estimated_velocity.tolist(),
            'last_confidence': self.last_confidence,
            'last_source': self.last_source
        }


# テスト用コード
if __name__ == "__main__":
    print("=== 位置フィルタテスト ===\n")

    # フィルタ作成
    filter = PositionFilter(
        window_size=5,
        outlier_threshold=0.1,
        enable_prediction=True
    )

    # テストデータ（正常→異常→正常のシナリオ）
    test_positions = [
        (0.0, 0.0, 1.0),   # 正常
        (0.01, 0.01, 1.0),  # 正常
        (0.02, 0.02, 1.0),  # 正常
        (0.5, 0.5, 1.0),    # 異常（急激な変化）
        (0.03, 0.03, 1.0),  # 正常に戻る
        (0.04, 0.04, 1.0),  # 正常
    ]

    marker_counts = [4, 4, 4, 2, 4, 4]  # マーカー数も変動

    print("テストデータ処理:")
    for i, (pos, markers) in enumerate(zip(test_positions, marker_counts)):
        result = filter.process_position(pos, markers)
        print(f"\nStep {i+1}:")
        print(f"  入力位置: {pos}")
        print(f"  マーカー数: {markers}")
        print(f"  フィルタ後: {result['filtered_position']}")
        print(f"  異常値: {result['is_outlier']}")
        print(f"  予測使用: {result['used_prediction']}")
        print(f"  信頼度: {result['confidence']:.2f}")
        time.sleep(0.01)

    # 統計表示
    print("\n" + "="*50)
    print("統計情報:")
    stats = filter.get_statistics()
    for key, value in stats.items():
        print(f"  {key}: {value}")
