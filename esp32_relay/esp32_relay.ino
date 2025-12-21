/*
 * ESP32-WROOM-32E 中継プログラム
 * PCからシリアルで角度指令を受信し、ESP-NOWでStampS3へ転送
 */

#include <esp_now.h>
#include <WiFi.h>

// StampS3のMACアドレス
//uint8_t broadcastAddress[] = {0x34, 0xB7, 0xDA, 0x5D, 0x27, 0x68}; // test dorone
//uint8_t broadcastAddress[] = {0x48, 0xCA, 0x43, 0x3A, 0x51, 0x30}; // drone1
//uint8_t broadcastAddress[] = {0x48, 0xCA, 0x43, 0x38, 0xA1, 0xCC}; // drone2
//uint8_t broadcastAddress[] = {0x48, 0xCA, 0x43, 0x38, 0xF0, 0x60}; // drone3
uint8_t broadcastAddress[] = {0x48, 0xCA, 0x43, 0x38, 0x9C, 0x88}; // droneX

// ESP-NOWピア情報
esp_now_peer_info_t peerInfo;

// 通信プロトコル定義
#define CMD_START    1
#define CMD_STOP     2
#define CMD_ANGLE    3
#define CMD_FEEDBACK 4

// シリアルバイナリプロトコル
const uint8_t SERIAL_ANGLE_HEADER = 'A';
const uint8_t SERIAL_FEEDBACK_HEADER = 'F';

// 受信バッファ
#define BUFFER_SIZE 64
// シリアル受信用の状態管理
static uint8_t binFrame[1 + sizeof(uint32_t) + sizeof(float) * 2 + 1];  // 14 byte
static size_t binIndex = 0;
static char textBuf[BUFFER_SIZE];
static size_t textIndex = 0;

// 送信データ構造体
struct AngleCommand {
  uint8_t command_type;  // コマンドタイプ
  uint32_t sequence;     // シーケンス番号
  float roll_ref;        // ロール角指令 [rad]
  float pitch_ref;       // ピッチ角指令 [rad]
  uint8_t checksum;      // チェックサム
} __attribute__((packed));

struct FeedbackPacket {
  uint8_t command_type;
  uint32_t sequence;
  float roll_ref;
  float pitch_ref;
  uint8_t checksum;
} __attribute__((packed));

// 統計情報
unsigned long packetsReceived = 0;
unsigned long packetsSent = 0;
unsigned long lastStatusTime = 0;
unsigned long feedbackPacketsReceived = 0;

// ESP-NOW送信コールバック（ESP-IDF v5.x対応）
#ifdef ESP_IDF_VERSION_MAJOR
  #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    // 新しいAPI (ESP-IDF 5.x)
    void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
      if (status == ESP_NOW_SEND_SUCCESS) {
        packetsSent++;
      }
    }
  #else
    // 古いAPI
    void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
      if (status == ESP_NOW_SEND_SUCCESS) {
        packetsSent++;
      }
    }
  #endif
#else
  // 古いAPI (レガシーサポート)
  void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
      packetsSent++;
    }
  }
#endif

void forwardFeedbackToHost(const FeedbackPacket& feedback) {
  uint8_t payload[sizeof(uint32_t) + sizeof(float) * 2];
  memcpy(&payload[0], &feedback.sequence, sizeof(uint32_t));
  memcpy(&payload[sizeof(uint32_t)], &feedback.roll_ref, sizeof(float));
  memcpy(&payload[sizeof(uint32_t) + sizeof(float)], &feedback.pitch_ref, sizeof(float));

  uint8_t checksum = calculateChecksum(payload, sizeof(payload));

  Serial.write(SERIAL_FEEDBACK_HEADER);
  Serial.write(payload, sizeof(payload));
  Serial.write(checksum);
}

#ifdef ESP_IDF_VERSION_MAJOR
  #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
      if (len != sizeof(FeedbackPacket)) {
        return;
      }

      const FeedbackPacket* feedback = reinterpret_cast<const FeedbackPacket*>(incomingData);
      uint8_t calc_checksum = calculateChecksum(incomingData, sizeof(FeedbackPacket) - 1);
      if (calc_checksum != feedback->checksum || feedback->command_type != CMD_FEEDBACK) {
        return;
      }

      feedbackPacketsReceived++;
      forwardFeedbackToHost(*feedback);
    }
  #else
    void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
      if (len != sizeof(FeedbackPacket)) {
        return;
      }

      const FeedbackPacket* feedback = reinterpret_cast<const FeedbackPacket*>(incomingData);
      uint8_t calc_checksum = calculateChecksum(incomingData, sizeof(FeedbackPacket) - 1);
      if (calc_checksum != feedback->checksum || feedback->command_type != CMD_FEEDBACK) {
        return;
      }

      feedbackPacketsReceived++;
      forwardFeedbackToHost(*feedback);
    }
  #endif
#else
  void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if (len != sizeof(FeedbackPacket)) {
      return;
    }

    const FeedbackPacket* feedback = reinterpret_cast<const FeedbackPacket*>(incomingData);
    uint8_t calc_checksum = calculateChecksum(incomingData, sizeof(FeedbackPacket) - 1);
    if (calc_checksum != feedback->checksum || feedback->command_type != CMD_FEEDBACK) {
      return;
    }

    feedbackPacketsReceived++;
    forwardFeedbackToHost(*feedback);
  }
#endif

void setup() {
  // シリアル初期化
  Serial.begin(115200);
  Serial.println("\n=================================");
  Serial.println("ESP32-WROOM-32E Relay Program");
  Serial.println("=================================");
  Serial.println("Waiting for commands from PC...");
  
  // WiFiをステーションモードに設定
  WiFi.mode(WIFI_STA);
  
  // ESP-NOW初期化
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // 送信コールバック登録
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // ピア情報設定
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // ピア追加
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.println("ESP-NOW Ready");
  Serial.print("Target MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", broadcastAddress[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println("\n");
}

void processSerialData() {
  const size_t ANGLE_FRAME_LENGTH = 1 + sizeof(uint32_t) + sizeof(float) * 2 + 1;

  while (Serial.available() > 0) {
    uint8_t b = Serial.read();

    // --- バイナリフレーム同期 ---
    if (b == SERIAL_ANGLE_HEADER) {
      // ヘッダ検出で新しいフレーム開始
      binIndex = 0;
      binFrame[binIndex++] = b;
      // テキストバッファは破棄して問題なし
      textIndex = 0;
      continue;
    }

    if (binIndex > 0 && binIndex < ANGLE_FRAME_LENGTH) {
      binFrame[binIndex++] = b;
      if (binIndex == ANGLE_FRAME_LENGTH) {
        // チェックサム検証
        uint8_t calc_checksum = 0;
        for (size_t i = 1; i < ANGLE_FRAME_LENGTH - 1; i++) {
          calc_checksum += binFrame[i];
        }

        if ((calc_checksum & 0xFF) == binFrame[ANGLE_FRAME_LENGTH - 1]) {
          // 角度データ取得
          uint32_t sequence = 0;
          float roll_ref = 0.0f;
          float pitch_ref = 0.0f;
          memcpy(&sequence, &binFrame[1], sizeof(uint32_t));
          memcpy(&roll_ref, &binFrame[1 + sizeof(uint32_t)], sizeof(float));
          memcpy(&pitch_ref, &binFrame[1 + sizeof(uint32_t) + sizeof(float)], sizeof(float));

          sendAngleCommand(roll_ref, pitch_ref, sequence);
          packetsReceived++;
        }
        // 次フレームに備えてリセット
        binIndex = 0;
      }
      continue;
    }

    // --- テキストコマンド処理（バイナリ同期中でない場合のみ） ---
    if (textIndex < BUFFER_SIZE - 1) {
      textBuf[textIndex++] = (char)b;
    }

    if (b == '\n') {
      textBuf[textIndex - 1] = '\0';  // 改行を終端に置き換え
      String cmd = String((char*)textBuf);
      cmd.trim();

      if (cmd == "start") {
        sendStartCommand();
      } else if (cmd == "stop") {
        sendStopCommand();
      }
      // 常にリセット
      textIndex = 0;
    }

    // バッファオーバーフロー対策
    if (textIndex >= BUFFER_SIZE - 1) {
      textIndex = 0;
    }
  }
}

void sendStartCommand() {
  AngleCommand cmd;
  cmd.command_type = CMD_START;
  cmd.sequence = 0;
  cmd.roll_ref = 0.0f;
  cmd.pitch_ref = 0.0f;
  cmd.checksum = calculateChecksum((uint8_t*)&cmd, sizeof(cmd) - 1);
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&cmd, sizeof(cmd));
  
  if (result == ESP_OK) {
    Serial.println("=> START command sent to StampS3");
  } else {
    Serial.println("Error sending START command");
  }
}

void sendStopCommand() {
  AngleCommand cmd;
  cmd.command_type = CMD_STOP;
  cmd.sequence = 0;
  cmd.roll_ref = 0.0f;
  cmd.pitch_ref = 0.0f;
  cmd.checksum = calculateChecksum((uint8_t*)&cmd, sizeof(cmd) - 1);
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&cmd, sizeof(cmd));
  
  if (result == ESP_OK) {
    Serial.println("=> STOP command sent to StampS3");
  } else {
    Serial.println("Error sending STOP command");
  }
}

void sendAngleCommand(float roll_ref, float pitch_ref, uint32_t sequence) {
  static unsigned long angleCounter = 0;
  AngleCommand cmd;
  cmd.command_type = CMD_ANGLE;
  cmd.sequence = sequence;
  cmd.roll_ref = roll_ref;
  cmd.pitch_ref = pitch_ref;
  cmd.checksum = calculateChecksum((uint8_t*)&cmd, sizeof(cmd) - 1);
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&cmd, sizeof(cmd));
  angleCounter++;
  
  // デバッグ出力（20回に1回 = 5Hz）
  if (angleCounter % 20 == 0) {
    float roll_deg = roll_ref * 180.0 / 3.14159;
    float pitch_deg = pitch_ref * 180.0 / 3.14159;
    Serial.printf("=> Angle[%lu]: R=%+6.1f° P=%+6.1f°", 
                  angleCounter, roll_deg, pitch_deg);
    if (result == ESP_OK) {
      Serial.println(" [OK]");
    } else {
      Serial.printf(" [FAIL:%d]\n", result);
    }
  }
}

uint8_t calculateChecksum(const uint8_t* data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return sum;
}

void printStatus() {
  unsigned long now = millis();
  if (now - lastStatusTime >= 5000) {  // 5秒ごと
    Serial.print("Status: RX=");
    Serial.print(packetsReceived);
    Serial.print(" TX=");
    Serial.print(packetsSent);
    Serial.print(" FB=");
    Serial.print(feedbackPacketsReceived);
    Serial.print(" Rate=");
    Serial.print((packetsReceived * 1000) / (now - lastStatusTime));
    Serial.println(" pkt/s");
    
    lastStatusTime = now;
  }
}

void loop() {
  // シリアルデータ処理
  processSerialData();
  
  // 定期的なステータス表示
  printStatus();
  
  // CPU負荷軽減
  delayMicroseconds(100);
}
