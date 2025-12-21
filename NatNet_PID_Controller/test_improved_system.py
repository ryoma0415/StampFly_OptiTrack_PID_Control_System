#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
改良版ホバリング制御システムのテストスクリプト
各コンポーネントの単体テストと統合テスト
"""

import sys
import os
import time
import numpy as np

# 親ディレクトリのパスを追加
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'python_controller'))

from position_filter import PositionFilter
from pid_controller import PIDController, XYPIDController


def test_position_filter():
    """位置フィルタのテスト"""
    print("\n" + "="*60)
    print("位置フィルタのテスト")
    print("="*60)

    # フィルタ作成
    filter = PositionFilter(
        window_size=5,
        outlier_threshold=0.1,
        enable_prediction=True
    )

    # テストシナリオ: 正常→異常（マーカー減少）→正常
    test_scenarios = [
        # (位置, マーカー数, 説明)
        ((0.00, 0.00, 1.0), 4, "初期位置（原点）"),
        ((0.01, 0.01, 1.0), 4, "小さな移動"),
        ((0.02, 0.02, 1.0), 4, "継続的な移動"),
        ((0.03, 0.03, 1.0), 3, "マーカー1個減少"),
        ((0.50, 0.50, 1.0), 2, "マーカー2個、位置が跳ねる（異常）"),
        ((0.04, 0.04, 1.0), 4, "マーカー復帰、正常位置"),
        ((0.05, 0.05, 1.0), 4, "正常継続"),
    ]

    for i, (pos, markers, description) in enumerate(test_scenarios):
        print(f"\nステップ {i+1}: {description}")

        # フィルタ処理
        result = filter.process_position(pos, markers)

        # 結果表示
        raw_x, raw_y, raw_z = result['raw_position']
        filt_x, filt_y, filt_z = result['filtered_position']

        print(f"  生データ: ({raw_x:.3f}, {raw_y:.3f}, {raw_z:.3f})")
        print(f"  フィルタ後: ({filt_x:.3f}, {filt_y:.3f}, {filt_z:.3f})")
        print(f"  マーカー数: {markers}")
        print(f"  異常値: {'はい' if result['is_outlier'] else 'いいえ'}")
        print(f"  予測使用: {'はい' if result['used_prediction'] else 'いいえ'}")
        print(f"  信頼度: {result['confidence']:.2%}")

        time.sleep(0.01)  # 時間経過をシミュレート

    # 統計表示
    stats = filter.get_statistics()
    print(f"\n統計:")
    print(f"  総サンプル数: {stats['total_samples']}")
    print(f"  異常値検出数: {stats['outlier_samples']}")
    print(f"  異常値率: {stats['outlier_rate']:.2%}")


def test_pid_controller():
    """拡張PID制御器のテスト"""
    print("\n" + "="*60)
    print("拡張PID制御器のテスト")
    print("="*60)

    # ユーザー指定のゲイン値を使用
    pid = PIDController(
        kp=0.139, ki=0.020, kd=0.204,
        output_limit=(-0.087, 0.087),
        d_filter_alpha=0.6,
        i_decay_rate=0.98,
        i_update_threshold=0.3,
        enable_i_control=True
    )

    # テストシナリオ: 位置誤差の変化
    print("\n通常動作のテスト（原点からのずれ）")
    position = 0.1  # 10cmのずれ

    for i in range(20):
        error = 0.0 - position  # 原点への誤差
        is_valid = True  # 正常データ

        # PID計算
        output = pid.calculate(error, is_data_valid=is_valid)

        # 簡易的な物理シミュレーション
        position += output * 0.05  # 制御出力による位置変化

        if i % 5 == 0:
            p, i_val, d = pid.get_components()
            print(f"  Step {i+1:2d}: 位置={position:+7.4f}m, "
                  f"誤差={error:+7.4f}m, 出力={output:+7.4f}rad")
            print(f"           P={p:+7.4f}, I={i_val:+7.4f}, D={d:+7.4f}")

        time.sleep(0.01)

    print("\n異常データ混入時のテスト")
    # I項をリセットして再開
    pid.reset()
    position = 0.1

    for i in range(20):
        error = 0.0 - position

        # ステップ10-12で異常データをシミュレート
        is_valid = not (10 <= i <= 12)

        if not is_valid:
            # 異常時は位置が跳ねる
            error = -0.5 if i == 11 else error
            pid.set_anomaly_state(True)
        else:
            pid.set_anomaly_state(False)

        output = pid.calculate(error, is_data_valid=is_valid)

        # 物理シミュレーション
        if is_valid:
            position += output * 0.05

        if i % 5 == 0 or not is_valid:
            p, i_val, d = pid.get_components()
            status = "正常" if is_valid else "異常"
            print(f"  Step {i+1:2d} [{status}]: 位置={position:+7.4f}m, "
                  f"誤差={error:+7.4f}m, 出力={output:+7.4f}rad")

        time.sleep(0.01)


def test_xy_pid_controller():
    """XY平面PID制御のテスト"""
    print("\n" + "="*60)
    print("XY平面PID制御のテスト")
    print("="*60)

    # XY制御器作成（ユーザー指定ゲイン）
    xy_pid = XYPIDController(
        kp_x=0.139, ki_x=0.020, kd_x=0.204,
        kp_y=-0.139, ki_y=-0.020, kd_y=-0.204,  # Y軸は符号反転
        output_limit=(-0.087, 0.087),
        d_filter_alpha=0.6,
        enable_i_control=True
    )

    # 初期位置（原点から離れた位置）
    pos_x, pos_y = 0.15, 0.10  # 15cm, 10cmのずれ

    print(f"初期位置: ({pos_x:.3f}, {pos_y:.3f}) m")
    print("目標: (0.000, 0.000) m\n")

    for i in range(30):
        # 誤差計算
        error_x = 0.0 - pos_x
        error_y = 0.0 - pos_y

        # 異常シミュレーション（ステップ15で一時的な異常）
        is_valid = not (i == 15)
        if not is_valid:
            xy_pid.set_anomaly_state(True)
            print(f"  ⚠ Step {i+1}: 異常検出（マーカー減少を想定）")
        else:
            xy_pid.set_anomaly_state(False)

        # PID計算
        roll_ref, pitch_ref = xy_pid.calculate(error_x, error_y, is_data_valid=is_valid)

        # 物理シミュレーション
        if is_valid:
            pos_x += roll_ref * 0.05
            pos_y += pitch_ref * 0.05

        # 表示（5ステップごと）
        if i % 5 == 0:
            roll_deg = roll_ref * 180 / np.pi
            pitch_deg = pitch_ref * 180 / np.pi
            distance = np.sqrt(pos_x**2 + pos_y**2)

            print(f"  Step {i+1:2d}: 位置=({pos_x:+6.3f}, {pos_y:+6.3f}) m, "
                  f"距離={distance:6.3f} m")
            print(f"           指令: Roll={roll_deg:+6.2f}°, Pitch={pitch_deg:+6.2f}°")

        time.sleep(0.01)

    # 最終位置
    final_distance = np.sqrt(pos_x**2 + pos_y**2)
    print(f"\n最終位置: ({pos_x:.3f}, {pos_y:.3f}) m")
    print(f"原点からの距離: {final_distance:.3f} m")


def test_integration():
    """統合テスト: フィルタ + PID制御"""
    print("\n" + "="*60)
    print("統合テスト: 位置フィルタ + PID制御")
    print("="*60)

    # コンポーネント初期化
    pos_filter = PositionFilter(
        window_size=5,
        outlier_threshold=0.1,
        enable_prediction=True
    )

    xy_pid = XYPIDController(
        kp_x=0.139, ki_x=0.020, kd_x=0.204,
        kp_y=-0.139, ki_y=-0.020, kd_y=-0.204,
        output_limit=(-0.087, 0.087),
        d_filter_alpha=0.6,
        enable_i_control=True
    )

    # シミュレーション: ノイズありの位置データ
    true_pos_x, true_pos_y = 0.2, 0.15  # 真の位置

    print("シナリオ: ノイズとマーカー減少のある環境での制御")
    print(f"初期位置: ({true_pos_x:.3f}, {true_pos_y:.3f}) m\n")

    for i in range(40):
        # マーカー数の変動をシミュレート
        if 15 <= i <= 18:
            marker_count = 2  # マーカー減少期間
            noise_level = 0.05  # ノイズ増大
        else:
            marker_count = 4
            noise_level = 0.005

        # 観測位置（ノイズあり）
        obs_x = true_pos_x + np.random.normal(0, noise_level)
        obs_y = true_pos_y + np.random.normal(0, noise_level)
        obs_z = 1.0

        # 異常値をたまに混入（マーカー減少時）
        if marker_count == 2 and np.random.random() < 0.3:
            obs_x += np.random.choice([-0.2, 0.2])
            obs_y += np.random.choice([-0.2, 0.2])

        # フィルタリング
        filter_result = pos_filter.process_position(
            (obs_x, obs_y, obs_z),
            marker_count=marker_count
        )

        filt_x, filt_y, filt_z = filter_result['filtered_position']
        is_outlier = filter_result['is_outlier']
        is_data_valid = not is_outlier

        # 誤差計算
        error_x = 0.0 - filt_x
        error_y = 0.0 - filt_y

        # PID制御
        if filter_result['consecutive_outliers'] > 3:
            xy_pid.set_anomaly_state(True)
        elif filter_result['consecutive_outliers'] == 0:
            xy_pid.set_anomaly_state(False)

        roll_ref, pitch_ref = xy_pid.calculate(
            error_x, error_y,
            is_data_valid=is_data_valid
        )

        # 物理更新（真の位置）
        if is_data_valid:
            true_pos_x += roll_ref * 0.03
            true_pos_y += pitch_ref * 0.03

        # 表示
        if i % 5 == 0 or is_outlier:
            status = "異常" if is_outlier else "正常"
            distance = np.sqrt(filt_x**2 + filt_y**2)

            print(f"Step {i+1:2d} [{status}]:")
            print(f"  観測: ({obs_x:+6.3f}, {obs_y:+6.3f}) マーカー={marker_count}")
            print(f"  フィルタ後: ({filt_x:+6.3f}, {filt_y:+6.3f}) 距離={distance:6.3f}m")

            if is_outlier:
                print(f"  ⚠ 異常値検出、予測使用: {filter_result['used_prediction']}")

        time.sleep(0.01)

    # 最終結果
    final_distance = np.sqrt(true_pos_x**2 + true_pos_y**2)
    print(f"\n最終位置: ({true_pos_x:.3f}, {true_pos_y:.3f}) m")
    print(f"原点からの距離: {final_distance:.3f} m")

    # フィルタ統計
    stats = pos_filter.get_statistics()
    print(f"\nフィルタ統計:")
    print(f"  異常値検出率: {stats['outlier_rate']:.2%}")
    print(f"  総サンプル数: {stats['total_samples']}")


def main():
    """メインテスト関数"""
    print("\n" + "="*60)
    print("改良版ホバリング制御システムのテスト")
    print("="*60)
    print("\nテスト項目:")
    print("1. 位置フィルタ")
    print("2. 拡張PID制御器")
    print("3. XY平面PID制御")
    print("4. 統合テスト")
    print("5. すべて実行")

    choice = input("\nテスト番号を選択 (1-5): ").strip()

    if choice == '1':
        test_position_filter()
    elif choice == '2':
        test_pid_controller()
    elif choice == '3':
        test_xy_pid_controller()
    elif choice == '4':
        test_integration()
    elif choice == '5':
        test_position_filter()
        test_pid_controller()
        test_xy_pid_controller()
        test_integration()
    else:
        print("無効な選択です")

    print("\n" + "="*60)
    print("テスト完了")
    print("="*60)


if __name__ == "__main__":
    main()