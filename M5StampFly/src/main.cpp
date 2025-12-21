/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include <FastLED.h>
#include "flight_control.hpp"
#include <esp_now.h>
#include <WiFi.h>
#include <cstring>

// コマンドタイプ定義
#define CMD_START    1
#define CMD_STOP     2
#define CMD_ANGLE    3
#define CMD_FEEDBACK 4

// 角度指令データ構造体
struct AngleCommand {
  uint8_t command_type;
  uint32_t sequence;
  float roll_ref;
  float pitch_ref;
  uint8_t checksum;
} __attribute__((packed));

struct AngleFeedback {
  uint8_t command_type;
  uint32_t sequence;
  float roll_ref;
  float pitch_ref;
  uint8_t checksum;
} __attribute__((packed));

// ESP-NOWコマンド受信用グローバル変数
volatile bool esp_now_command_received = false;
volatile char received_command[32] = {0};

// 角度指令受信用グローバル変数
volatile float target_roll_angle = 0.0f;   // ロール角指令値 [rad]
volatile float target_pitch_angle = 0.0f;  // ピッチ角指令値 [rad]
volatile bool angle_command_active = false; // 角度制御モード有効フラグ
volatile uint32_t last_angle_command_time = 0; // 最後に角度指令を受信した時刻
volatile uint32_t target_angle_sequence = 0;   // 最新角度指令のシーケンス番号

// ESP32リレーのMACアドレス（StampFly -> Relayのフィードバック送信用）
const uint8_t relay_mac_address[6] = {0x88, 0x13, 0xBF, 0x02, 0xEF, 0xF0};
static esp_now_peer_info_t relayPeerInfo = {};

uint8_t calculateChecksum(const uint8_t* data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum;
}

bool send_angle_feedback(float roll_cmd, float pitch_cmd, uint32_t sequence) {
  AngleFeedback feedback;
  feedback.command_type = CMD_FEEDBACK;
  feedback.sequence = sequence;
  feedback.roll_ref = roll_cmd;
  feedback.pitch_ref = pitch_cmd;
  feedback.checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(&feedback), sizeof(feedback) - 1);

  esp_err_t result = esp_now_send(relay_mac_address, reinterpret_cast<uint8_t*>(&feedback), sizeof(feedback));
  return (result == ESP_OK);
}

// ESP32のバージョンに応じた受信コールバック関数
#ifdef ESP_IDF_VERSION_MAJOR // ESP-IDF v4.x以降の検出
  #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    // 新しいAPI (ESP-IDF 5.x)
    void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
      // 構造体データとして処理
      if (len == sizeof(AngleCommand)) {
        AngleCommand* cmd = (AngleCommand*)incomingData;
        
        // チェックサム検証
        uint8_t calc_checksum = 0;
        for (int i = 0; i < sizeof(AngleCommand) - 1; i++) {
          calc_checksum += incomingData[i];
        }
        
        if (calc_checksum == cmd->checksum) {
          switch (cmd->command_type) {
            case CMD_START:
              strcpy((char*)received_command, "start");
              esp_now_command_received = true;
              angle_command_active = false;
              USBSerial.println("Command: START");
              break;
              
            case CMD_STOP:
              strcpy((char*)received_command, "stop");
              esp_now_command_received = true;
              angle_command_active = false;
              USBSerial.println("Command: STOP");
              break;
              
            case CMD_ANGLE:
              target_roll_angle = cmd->roll_ref;
              target_pitch_angle = cmd->pitch_ref;
              target_angle_sequence = cmd->sequence;
              angle_command_active = true;
              last_angle_command_time = millis();
              // デバッグ出力は頻度を下げる
              static uint32_t debug_counter = 0;
              if (++debug_counter % 100 == 0) {
                USBSerial.printf("Angle: R=%.3f, P=%.3f rad\n", 
                               target_roll_angle, target_pitch_angle);
              }
              break;
          }
        }
      }
    }
  #else
    // 古いAPI
    void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
      // 構造体データとして処理
      if (len == sizeof(AngleCommand)) {
        AngleCommand* cmd = (AngleCommand*)incomingData;
        
        // チェックサム検証
        uint8_t calc_checksum = 0;
        for (int i = 0; i < sizeof(AngleCommand) - 1; i++) {
          calc_checksum += incomingData[i];
        }
        
        if (calc_checksum == cmd->checksum) {
          switch (cmd->command_type) {
            case CMD_START:
              strcpy((char*)received_command, "start");
              esp_now_command_received = true;
              angle_command_active = false;
              USBSerial.println("Command: START");
              break;
              
            case CMD_STOP:
              strcpy((char*)received_command, "stop");
              esp_now_command_received = true;
              angle_command_active = false;
              USBSerial.println("Command: STOP");
              break;
              
            case CMD_ANGLE:
              target_roll_angle = cmd->roll_ref;
              target_pitch_angle = cmd->pitch_ref;
              target_angle_sequence = cmd->sequence;
              angle_command_active = true;
              last_angle_command_time = millis();
              // デバッグ出力は頻度を下げる
              static uint32_t debug_counter = 0;
              if (++debug_counter % 100 == 0) {
                USBSerial.printf("Angle: R=%.3f, P=%.3f rad\n", 
                               target_roll_angle, target_pitch_angle);
              }
              break;
          }
        }
      }
    }
  #endif
#else
  // 古いAPI (レガシーサポート)
  void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    // 構造体データとして処理
    if (len == sizeof(AngleCommand)) {
      AngleCommand* cmd = (AngleCommand*)incomingData;
      
      // チェックサム検証
      uint8_t calc_checksum = 0;
      for (int i = 0; i < sizeof(AngleCommand) - 1; i++) {
        calc_checksum += incomingData[i];
      }
      
      if (calc_checksum == cmd->checksum) {
        switch (cmd->command_type) {
          case CMD_START:
            strcpy((char*)received_command, "start");
            esp_now_command_received = true;
            angle_command_active = false;
            USBSerial.println("Command: START");
            break;
            
          case CMD_STOP:
            strcpy((char*)received_command, "stop");
            esp_now_command_received = true;
            angle_command_active = false;
            USBSerial.println("Command: STOP");
            break;
            
          case CMD_ANGLE:
            target_roll_angle = cmd->roll_ref;
            target_pitch_angle = cmd->pitch_ref;
            target_angle_sequence = cmd->sequence;
            angle_command_active = true;
            last_angle_command_time = millis();
            // デバッグ出力は頻度を下げる
            static uint32_t debug_counter = 0;
            if (++debug_counter % 100 == 0) {
              USBSerial.printf("Angle: R=%.3f, P=%.3f rad\n", 
                             target_roll_angle, target_pitch_angle);
            }
            break;
        }
      }
    }
  }
#endif

// VL53L0X_ADDRESS           0x29
// MPU6886_ADDRESS           0x68
// BMP280_ADDRESS            0x76

void setup() {
    init_copter();
    delay(100);
    
    // ESP-NOW初期化
    USBSerial.println("Initializing ESP-NOW...");
    
    WiFi.mode(WIFI_STA);
    USBSerial.print("MAC Address: ");
    USBSerial.println(WiFi.macAddress());
    
    if (esp_now_init() != ESP_OK) {
        USBSerial.println("Error initializing ESP-NOW");
        return;
    }
    
    // 受信コールバック関数の登録
    esp_now_register_recv_cb(OnDataRecv);

    memset(&relayPeerInfo, 0, sizeof(relayPeerInfo));
    memcpy(relayPeerInfo.peer_addr, relay_mac_address, 6);
    relayPeerInfo.channel = 0;
    relayPeerInfo.encrypt = false;

    esp_err_t peer_result = esp_now_add_peer(&relayPeerInfo);
    if (peer_result == ESP_OK) {
        USBSerial.println("Relay peer registered for feedback");
    } else if (peer_result == ESP_ERR_ESPNOW_EXIST) {
        USBSerial.println("Relay peer already registered");
    } else {
        USBSerial.printf("Failed to add relay peer: %d\n", peer_result);
    }
    USBSerial.println("ESP-NOW Ready. Waiting for commands...");
    USBSerial.println("Hovering Control Mode Active");
}

void loop() {
    loop_400Hz();
}
