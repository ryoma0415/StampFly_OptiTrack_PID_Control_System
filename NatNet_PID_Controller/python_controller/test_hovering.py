#!/usr/bin/env python3
"""
ホバリング制御のテストプログラム
位置データ取得と角度指令生成の動作確認用
"""

import time
import sys
import os

# 親ディレクトリのパスを追加
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
project6_dir = os.path.join(parent_dir, '..', 'Claude-Code-006_Project6')
if os.path.exists(project6_dir):
    sys.path.insert(0, os.path.abspath(project6_dir))

from hovering_controller import HoveringController

class TestHoveringController(HoveringController):
    """テスト用の拡張コントローラ"""
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.test_mode = True
        
    def send_angle_command(self, roll_ref, pitch_ref):
        """角度指令送信（テストモードでは詳細表示）"""
        # 角度を度に変換して表示
        roll_deg = roll_ref * 180 / 3.14159
        pitch_deg = pitch_ref * 180 / 3.14159
        
        print(f"[SEND] Roll: {roll_deg:+6.2f}°, Pitch: {pitch_deg:+6.2f}°")
        
        # 実際の送信も行う
        return super().send_angle_command(roll_ref, pitch_ref)
        
    def test_position_only(self):
        """位置データ取得のみテスト（飛行なし）"""
        print("\n=== 位置データ取得テスト ===")
        print("Motiveでマーカーが認識されているか確認してください。")
        print("Ctrl+Cで終了\n")
        
        # NatNet接続
        print("Optitrack/Motiveに接続中...")
        from NatNetClient import NatNetClient
        self.natnet_client = NatNetClient()
        self.natnet_client.set_server_address("127.0.0.1")
        self.natnet_client.set_client_address("127.0.0.1")
        self.natnet_client.set_use_multicast(True)
        
        # コールバック登録（drone_tracker.pyと同じ）
        self.natnet_client.new_frame_with_data_listener = self.receive_mocap_frame
        self.natnet_client.set_print_level(0)
        
        # NatNet開始
        if not self.natnet_client.run('d'):
            print("✗ 接続に失敗しました")
            return
            
        time.sleep(1)
        
        if not self.natnet_client.connected():
            print("✗ Motiveが見つかりません")
            return
            
        # データ記述を要求
        self.natnet_client.send_request(
            self.natnet_client.command_socket,
            self.natnet_client.NAT_REQUEST_MODELDEF,
            "",
            ("127.0.0.1", self.natnet_client.command_port)
        )
        
        print("✓ Optitrack接続完了")
        print("\n位置データ監視中...\n")
        
        try:
            while True:
                if self.current_position:
                    pos_x, pos_y = self.current_position
                    error_x = self.target_position[0] - pos_x
                    error_y = self.target_position[1] - pos_y
                    
                    # PID計算（テスト用）
                    roll_ref, pitch_ref = self.pid_controller.calculate(
                        error_x, error_y, time.time()
                    )
                    
                    roll_deg = roll_ref * 180 / 3.14159
                    pitch_deg = pitch_ref * 180 / 3.14159
                    
                    print(f"位置: X={pos_x:+6.3f}, Y={pos_y:+6.3f} m | "
                          f"誤差: X={error_x:+6.3f}, Y={error_y:+6.3f} m | "
                          f"指令: R={roll_deg:+6.2f}°, P={pitch_deg:+6.2f}°")
                else:
                    print("位置データなし - マーカーを確認してください")
                    
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\n\nテスト終了")
            
        finally:
            if self.natnet_client:
                self.natnet_client.shutdown()

def main():
    """メイン関数"""
    print("ホバリング制御テストプログラム")
    print("1: 位置データ取得テスト（飛行なし）")
    print("2: 通常の飛行テスト")
    print("q: 終了")
    
    choice = input("\n選択 > ").strip()
    
    if choice == "1":
        controller = TestHoveringController()
        controller.test_position_only()
    elif choice == "2":
        controller = TestHoveringController()
        controller.run()
    else:
        print("終了します")

if __name__ == "__main__":
    main()