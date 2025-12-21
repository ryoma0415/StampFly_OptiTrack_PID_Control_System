#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PIDコントローラ（拡張版）
ドローンの位置制御用PIDコントローラ実装
- D項の低域通過フィルタ
- I項の条件付き制御
- 異常時のI項減衰機能
"""

import time
import numpy as np


class PIDController:
    """
    拡張PIDコントローラクラス

    P: 比例項 - 現在の誤差に比例
    I: 積分項 - 誤差の累積に比例（定常偏差を除去）
    D: 微分項 - 誤差の変化率に比例（オーバーシュートを抑制）

    拡張機能:
    - D項の低域通過フィルタ
    - I項の条件付き更新と減衰
    - 異常検出時の特別処理
    """

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limit=None,
                 d_filter_alpha=0.7, i_decay_rate=0.98,
                 i_update_threshold=0.5, enable_i_control=True):
        """
        PIDコントローラの初期化

        Args:
            kp: 比例ゲイン
            ki: 積分ゲイン
            kd: 微分ゲイン
            output_limit: 出力制限値 (min, max) のタプル
            d_filter_alpha: D項の低域通過フィルタ係数 (0-1、大きいほど応答が速い)
            i_decay_rate: I項の減衰率 (0-1、異常時に使用)
            i_update_threshold: I項を更新する誤差閾値
            enable_i_control: I項の条件付き制御を有効化
        """
        # PIDゲイン
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # 出力制限
        self.output_limit = output_limit

        # 内部状態
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

        # デバッグ用の各項の値
        self.last_p_term = 0.0
        self.last_i_term = 0.0
        self.last_d_term = 0.0

        # 拡張機能のパラメータ
        self.d_filter_alpha = d_filter_alpha
        self.i_decay_rate = i_decay_rate
        self.i_update_threshold = i_update_threshold
        self.enable_i_control = enable_i_control

        # D項フィルタ用の履歴
        self.filtered_derivative = 0.0

        # I項制御用のフラグ
        self.i_update_suspended = False
        self.anomaly_detected = False
        self.anomaly_recovery_count = 0
        
    def reset(self):
        """内部状態をリセット"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        self.last_p_term = 0.0
        self.last_i_term = 0.0
        self.last_d_term = 0.0
        # 拡張機能の状態もリセット
        self.filtered_derivative = 0.0
        self.i_update_suspended = False
        self.anomaly_detected = False
        self.anomaly_recovery_count = 0

    def soft_reset_integral(self, factor=0.5):
        """
        I項をソフトリセット（部分的に減少）

        Args:
            factor: 減少係数 (0-1、0で完全リセット、1で変化なし)
        """
        self.integral *= factor
        self.last_i_term = self.ki * self.integral

    def set_anomaly_state(self, is_anomaly):
        """
        異常状態を設定

        Args:
            is_anomaly: 異常状態フラグ
        """
        if is_anomaly and not self.anomaly_detected:
            # 異常開始時
            self.anomaly_detected = True
            self.i_update_suspended = True
            self.anomaly_recovery_count = 0
        elif not is_anomaly and self.anomaly_detected:
            # 異常から回復
            self.anomaly_detected = False
            self.anomaly_recovery_count = 10  # 回復期間を設定
        
    def set_gains(self, kp=None, ki=None, kd=None):
        """
        PIDゲインを設定
        
        Args:
            kp: 比例ゲイン（Noneの場合は変更なし）
            ki: 積分ゲイン（Noneの場合は変更なし）
            kd: 微分ゲイン（Noneの場合は変更なし）
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
            # 積分ゲインが変更されたら積分値をリセット
            if ki == 0:
                self.integral = 0.0
        if kd is not None:
            self.kd = kd
            
    def calculate(self, error, current_time=None, is_data_valid=True):
        """
        PID制御出力を計算（拡張版）

        Args:
            error: 目標値と現在値の誤差
            current_time: 現在時刻（Noneの場合は自動取得）
            is_data_valid: データの有効性フラグ（異常検出時はFalse）

        Returns:
            float: PID制御出力値
        """
        # 時刻の取得
        if current_time is None:
            current_time = time.time()

        # 初回呼び出し時の処理
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            dt = 0.01  # 初回は仮の値
        else:
            dt = current_time - self.prev_time

        # dt が小さすぎる場合はスキップ（ゼロ除算防止）
        if dt < 0.001:
            return self.last_p_term + self.last_i_term + self.last_d_term

        # P項（比例項）
        p_term = self.kp * error
        self.last_p_term = p_term

        # I項（積分項）の条件付き更新
        if self.enable_i_control:
            # 異常回復期間中の処理
            if self.anomaly_recovery_count > 0:
                # I項を徐々に減衰
                self.integral *= self.i_decay_rate
                self.anomaly_recovery_count -= 1
                if self.anomaly_recovery_count == 0:
                    self.i_update_suspended = False

            # I項の更新条件
            should_update_i = (
                not self.i_update_suspended and
                is_data_valid and
                abs(error) < self.i_update_threshold  # 大きな誤差時は更新しない
            )

            if should_update_i:
                # I項を更新
                self.integral += error * dt

                # より厳格なアンチワインドアップ
                if self.output_limit is not None and self.ki != 0:
                    # 動的な積分制限（誤差が大きいほど制限を厳しく）
                    error_factor = np.exp(-abs(error) * 2)  # 誤差が大きいと小さくなる
                    max_integral = abs(self.output_limit[1] / self.ki) * error_factor
                    self.integral = np.clip(self.integral, -max_integral, max_integral)
            elif not is_data_valid and self.enable_i_control:
                # データ無効時はI項を少し減衰
                self.integral *= 0.995

        else:
            # 従来のI項計算
            self.integral += error * dt
            if self.output_limit is not None and self.ki != 0:
                max_integral = abs(self.output_limit[1] / self.ki)
                self.integral = max(-max_integral, min(max_integral, self.integral))

        i_term = self.ki * self.integral
        self.last_i_term = i_term

        # D項（微分項）with フィルタリング
        if dt > 0:
            raw_derivative = (error - self.prev_error) / dt

            # 低域通過フィルタを適用
            self.filtered_derivative = (
                self.d_filter_alpha * raw_derivative +
                (1 - self.d_filter_alpha) * self.filtered_derivative
            )

            # 異常時はD項をさらに抑制
            if not is_data_valid:
                derivative = self.filtered_derivative * 0.3
            else:
                derivative = self.filtered_derivative
        else:
            derivative = 0.0

        d_term = self.kd * derivative
        self.last_d_term = d_term

        # PID出力の計算
        output = p_term + i_term + d_term

        # 出力制限
        if self.output_limit is not None:
            output = max(self.output_limit[0], min(self.output_limit[1], output))

        # 状態の更新
        self.prev_error = error
        self.prev_time = current_time

        return output
        
    def get_components(self):
        """
        PID各項の値を取得（デバッグ用）
        
        Returns:
            tuple: (P項, I項, D項)
        """
        return (self.last_p_term, self.last_i_term, self.last_d_term)


class XYPIDController:
    """
    XY平面用の拡張PIDコントローラ（X軸とY軸それぞれ独立）
    """

    def __init__(self,
                 kp_x=1.0, ki_x=0.0, kd_x=0.0,
                 kp_y=1.0, ki_y=0.0, kd_y=0.0,
                 output_limit=None,
                 d_filter_alpha=0.7,
                 i_decay_rate=0.98,
                 i_update_threshold=0.5,
                 enable_i_control=True):
        """
        XY平面PIDコントローラの初期化（拡張版）

        Args:
            kp_x, ki_x, kd_x: X軸のPIDゲイン
            kp_y, ki_y, kd_y: Y軸のPIDゲイン
            output_limit: 出力制限値 (min, max) のタプル
            d_filter_alpha: D項フィルタ係数
            i_decay_rate: I項減衰率
            i_update_threshold: I項更新閾値
            enable_i_control: I項制御の有効化
        """
        self.pid_x = PIDController(
            kp_x, ki_x, kd_x, output_limit,
            d_filter_alpha, i_decay_rate,
            i_update_threshold, enable_i_control
        )
        self.pid_y = PIDController(
            kp_y, ki_y, kd_y, output_limit,
            d_filter_alpha, i_decay_rate,
            i_update_threshold, enable_i_control
        )

    def reset(self):
        """両軸のPIDをリセット"""
        self.pid_x.reset()
        self.pid_y.reset()

    def soft_reset_integral(self, factor=0.5):
        """両軸のI項をソフトリセット"""
        self.pid_x.soft_reset_integral(factor)
        self.pid_y.soft_reset_integral(factor)

    def set_anomaly_state(self, is_anomaly):
        """両軸の異常状態を設定"""
        self.pid_x.set_anomaly_state(is_anomaly)
        self.pid_y.set_anomaly_state(is_anomaly)

    def calculate(self, error_x, error_y, current_time=None, is_data_valid=True):
        """
        XY両軸のPID制御出力を計算（拡張版）

        Args:
            error_x: X軸の誤差
            error_y: Y軸の誤差
            current_time: 現在時刻
            is_data_valid: データ有効性フラグ

        Returns:
            tuple: (X軸出力, Y軸出力)
        """
        output_x = self.pid_x.calculate(error_x, current_time, is_data_valid)
        output_y = self.pid_y.calculate(error_y, current_time, is_data_valid)
        return (output_x, output_y)
        
    def get_all_components(self):
        """
        全てのPID成分を取得
        
        Returns:
            dict: X軸とY軸のP, I, D項
        """
        x_components = self.pid_x.get_components()
        y_components = self.pid_y.get_components()
        return {
            'x': {'p': x_components[0], 'i': x_components[1], 'd': x_components[2]},
            'y': {'p': y_components[0], 'i': y_components[1], 'd': y_components[2]}
        }


# テスト用コード
if __name__ == "__main__":
    print("=== PIDコントローラテスト ===\n")
    
    # 単軸PIDテスト
    pid = PIDController(kp=1.0, ki=0.1, kd=0.01, output_limit=(-10, 10))
    
    print("単軸PIDテスト（目標: 0, 初期位置: 5）")
    position = 5.0
    for i in range(10):
        error = 0.0 - position  # 目標位置0からの誤差
        output = pid.calculate(error)
        position += output * 0.1  # 簡易的なシミュレーション
        p, i, d = pid.get_components()
        print(f"  Step {i+1}: 位置={position:6.3f}, 誤差={error:6.3f}, "
              f"出力={output:6.3f} (P={p:5.2f}, I={i:5.2f}, D={d:5.2f})")
        time.sleep(0.01)
    
    print("\n" + "="*50)
    print("\nXY平面PIDテスト")
    
    # XY平面PIDテスト
    xy_pid = XYPIDController(
        kp_x=1.5, ki_x=0.1, kd_x=0.05,
        kp_y=1.5, ki_y=0.1, kd_y=0.05,
        output_limit=(-5, 5)
    )
    
    # 初期位置 (2, 3) から原点 (0, 0) へ
    pos_x, pos_y = 2.0, 3.0
    print(f"初期位置: ({pos_x:.2f}, {pos_y:.2f})")
    print("目標位置: (0.00, 0.00)\n")
    
    for i in range(10):
        error_x = 0.0 - pos_x
        error_y = 0.0 - pos_y
        output_x, output_y = xy_pid.calculate(error_x, error_y)
        
        # 位置の更新（簡易シミュレーション）
        pos_x += output_x * 0.1
        pos_y += output_y * 0.1
        
        components = xy_pid.get_all_components()
        print(f"Step {i+1}:")
        print(f"  位置: ({pos_x:6.3f}, {pos_y:6.3f})")
        print(f"  誤差: ({error_x:6.3f}, {error_y:6.3f})")
        print(f"  出力: ({output_x:6.3f}, {output_y:6.3f})")
        time.sleep(0.01)
    
    print("\nテスト完了")