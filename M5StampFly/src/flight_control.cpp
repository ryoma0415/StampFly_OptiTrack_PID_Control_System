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

//
// StampFly Flight Control Main Module
//
// Desigend by Kouhei Ito 2023~2024
//
// 2024-06-20 高度制御改良　段差対応
// 2024-06-25 高度制御改良　上昇持続バグ修正
// 2024-06-29 自動離陸追加
// 2024-06-29 自動着陸追加
// 2024-06-29 送信機OFFで自動着陸
// 2024-06-29 着陸時、Madgwick Filter Off
// 2024-07-21 flip関数追加、高度センサの測定限界で自動降下（暫定版）
// 2024-08-10 Acroモードで高度制御働かないバグを修正

#include "flight_control.hpp"
#include "pid.hpp"
#include "sensor.hpp"
#include <string.h>  // strcmpのため


// モータPWM出力Pinのアサイン
// Motor PWM Pin
const int pwmFrontLeft  = 5;
const int pwmFrontRight = 42;
const int pwmRearLeft   = 10;
const int pwmRearRight  = 41;

// モータPWM周波数
// Motor PWM Frequency
const int freq = 150000;

// PWM分解能
// PWM Resolution
const int resolution = 8;

// モータチャンネルのアサイン
// Motor Channel
const int FrontLeft_motor  = 0;
const int FrontRight_motor = 1;
const int RearLeft_motor   = 2;
const int RearRight_motor  = 3;

// 制御周期
// Control period
float Control_period = 0.0025f;  // 400Hz

// 姿勢バイアス補正値（ドローン個体差による調整用）
// Attitude bias correction (for individual drone adjustment)
// drone X : (Roll, Pitch) = (0.0f, 0.0f)
// drone test : (Roll, Pitch) = (-0.005f, -0.03f)
// drone 1 : (Roll, Pitch) = (-0.01f, -0.01f)
// drone 2 : (Roll, Pitch) = (-0.01f, -0.01f)
// drone 3 : (Roll, Pitch) = (0.0f, 0.0f)
const float Roll_angle_bias = 0.0f;
const float Pitch_angle_bias = 0.0f;

// PID Gain
// Rate control PID gain
const float Roll_rate_kp  = 0.65f;
const float Roll_rate_ti  = 0.7f;
const float Roll_rate_td  = 0.01;
const float Roll_rate_eta = 0.125f;

const float Pitch_rate_kp  = 0.95f;
const float Pitch_rate_ti  = 0.7f;
const float Pitch_rate_td  = 0.025f;
const float Pitch_rate_eta = 0.125f;

const float Yaw_rate_kp  = 3.0f;
const float Yaw_rate_ti  = 0.8f;
const float Yaw_rate_td  = 0.01f;
const float Yaw_rate_eta = 0.125f;

// Angle control PID gain
const float Rall_angle_kp  = 5.0f;  // 8.0
const float Rall_angle_ti  = 4.0f;
const float Rall_angle_td  = 0.04f;
const float Rall_angle_eta = 0.125f;

const float Pitch_angle_kp  = 5.0f;  // 8.0
const float Pitch_angle_ti  = 4.0f;
const float Pitch_angle_td  = 0.04f;
const float Pitch_angle_eta = 0.125f;

// Altitude control PID gain
const float alt_kp     = 0.38f;  // 5.0//soso 0.5
const float alt_ti     = 10.0f;  // 200.0//soso 10.0
const float alt_td     = 0.5f;   // 0.5//soso 0.5
const float alt_eta    = 0.125f;
const float alt_period = 0.0333;

const float z_dot_kp  = 0.08f;  // 0.35//soso 0.1
const float z_dot_ti  = 0.95f;  // 500.0//soso 0.95
const float z_dot_td  = 0.08f;  // 0.15//1.0//soso 0.08
const float z_dot_eta = 0.125f;

const float Duty_bias_up   = 1.581f;  // Altitude Control parameter　Itolab 1.589 M5Stack 1.581
const float Duty_bias_down = 1.578f;  // Auto landing  parameter Itolab 1.578 M5Stack 1.578

// Times
volatile float Elapsed_time     = 0.0f;
volatile float Old_Elapsed_time = 0.0f;
volatile float Interval_time    = 0.0f;
volatile uint32_t S_time = 0, E_time = 0, D_time = 0, Dt_time = 0;

// Counter
uint8_t AngleControlCounter   = 0;
uint16_t RateControlCounter   = 0;
uint16_t OffsetCounter        = 0;
uint16_t Auto_takeoff_counter = 0;

// Motor Duty
volatile float FrontRight_motor_duty = 0.0f;
volatile float FrontLeft_motor_duty  = 0.0f;
volatile float RearRight_motor_duty  = 0.0f;
volatile float RearLeft_motor_duty   = 0.0f;

// 制御目標
// PID Control reference
// 角速度目標値
// Rate reference
volatile float Roll_rate_reference = 0.0f, Pitch_rate_reference = 0.0f, Yaw_rate_reference = 0.0f;
// 角度目標値
// Angle reference
volatile float Roll_angle_reference = 0.0f, Pitch_angle_reference = 0.0f, Yaw_angle_reference = 0.0f;
// 舵角指令値
// Commanad
// スロットル指令値
// Throttle
volatile float Thrust_command = 0.0f, Thrust_command2 = 0.0f;
// 角速度指令値
// Rate command
volatile float Roll_rate_command = 0.0f, Pitch_rate_command = 0.0f, Yaw_rate_command = 0.0f;
// 角度指令値
// Angle comannd
volatile float Roll_angle_command = 0.0f, Pitch_angle_command = 0.0f, Yaw_angle_command = 0.0f;

// Offset
volatile float Roll_angle_offset = 0.0f, Pitch_angle_offset = 0.0f, Yaw_angle_offset = 0.0f;
volatile float Elevator_center = 0.0f, Aileron_center = 0.0f, Rudder_center = 0.0f;

// Machine state & flag
float Timevalue          = 0.0f;
float Motor_on_duty_threshold         = 0.1f;
float Angle_control_on_duty_threshold = 0.5f;
int8_t BtnA_counter                   = 0;
uint8_t BtnA_on_flag                  = 0;
uint8_t BtnA_off_flag                 = 1;
volatile uint8_t Loop_flag            = 0;
uint8_t Landing_state         = 0;
uint8_t OladRange0flag        = 0;

// 自動飛行シーケンス用変数
AutoFlightState auto_state = AUTO_INIT;
AutoFlightState old_auto_state = AUTO_INIT;  // OldModeの代替
uint32_t auto_timer = 0;
uint32_t phase_start_time = 0;
const uint32_t WAIT_TIME = 3000;     // 待機時間 3秒（センサーキャリブレーション待機）
// HOVER_TIMEは削除（無限ホバリングのため不要）
const uint32_t MAX_FLIGHT_TIME = 120000; // 最大飛行時間 120秒（安全のため2分に延長）
// dreoneX : 0.2f
// drone test : 0.2f
const float TARGET_ALTITUDE = 0.2f;  // 目標高度 50cm

// PID object and etc.
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;
// PID alt;
PID alt_pid;
PID z_dot_pid;
Filter Thrust_filtered;
Filter Duty_fr;
Filter Duty_fl;
Filter Duty_rr;
Filter Duty_rl;

volatile float Thrust0 = 0.0;
uint8_t Alt_flag       = 0;

// 速度目標Z
float Z_dot_ref = 0.0f;

// 高度目標
const float Alt_ref0   = 0.5f;
volatile float Alt_ref = Alt_ref0;

uint8_t ahrs_reset_flag      = 0;
uint8_t last_ahrs_reset_flag = 0;

// Function declaration
void init_pwm();
void control_init();
void get_command(void);
void angle_control(void);
void rate_control(void);
void motor_stop(void);
void reset_rate_control(void);
void reset_angle_control(void);
uint8_t auto_landing(void);
float get_trim_duty(float voltage);

// 割り込み関数
// Intrupt function
hw_timer_t* timer = NULL;
void IRAM_ATTR onTimer() {
    Loop_flag = 1;
}

// Initialize Multi copter
void init_copter(void) {
    auto_state = AUTO_INIT;  // 初期状態設定

    // Initialize Serial communication
    USBSerial.begin(115200);
    delay(1500);
    USBSerial.printf("Start StampFly!\r\n");

    // Initialize PWM
    init_pwm();
    sensor_init();
    USBSerial.printf("Finish sensor init!\r\n");

    // PID GAIN and etc. Init
    control_init();

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 2500, true);
    timerAlarmEnable(timer);

    USBSerial.printf("Finish StampFly init!\r\n");
    USBSerial.printf("Enjoy Flight!\r\n");
}

// Main loop
void loop_400Hz(void) {
    static uint8_t led = 1;
    float sense_time;
    // 割り込みにより400Hzで以降のコードが実行
    while (Loop_flag == 0);
    Loop_flag = 0;

    E_time           = micros();
    Old_Elapsed_time = Elapsed_time;
    Elapsed_time     = 1e-6 * (E_time - S_time);
    Interval_time    = Elapsed_time - Old_Elapsed_time;
    Timevalue += 0.0025f;

    // Read Sensor Value
    sense_time       = sensor_read();
    uint32_t cs_time = micros();

    // Begin state machine using auto_state
    if (auto_state == AUTO_INIT) {
        motor_stop();
        Elevator_center    = 0.0f;
        Aileron_center     = 0.0f;
        Rudder_center      = 0.0f;
        Roll_angle_offset  = 0.0f;
        Pitch_angle_offset = 0.0f;
        Yaw_angle_offset   = 0.0f;
        sensor_reset_offset();
        auto_state = AUTO_CALIBRATION;
        return;
    } else if (auto_state == AUTO_CALIBRATION) {
        motor_stop();
        // Gyro offset Estimate
        if (OffsetCounter < AVERAGENUM) {
            sensor_calc_offset_avarage();
            OffsetCounter++;
            return;
        }
        // State change
        auto_state = AUTO_WAIT;
        S_time = micros();
        auto_timer = millis();
        phase_start_time = millis();
        USBSerial.println("Auto flight: Waiting...");
        return;
    } else if (auto_state == AUTO_TAKEOFF || auto_state == AUTO_HOVER) {
        Control_period = Interval_time;

        // 自動飛行シーケンス制御
        if (auto_state == AUTO_TAKEOFF) {
            // 離陸中 - 50cmまで上昇
            if (Altitude2 >= TARGET_ALTITUDE - 0.05f) {
                auto_state = AUTO_HOVER;
                phase_start_time = millis();
                USBSerial.printf("Auto flight: Hovering at %.2f m\n", Altitude2);
            }
        } else if (auto_state == AUTO_HOVER) {
            // ホバリング中 - stopコマンド待機
            // ESP-NOWコマンドチェック
            if (esp_now_command_received) {
                if (strcmp((char*)received_command, "stop") == 0) {
                    esp_now_command_received = false;  // フラグリセット
                    auto_state = AUTO_LANDING;
                    USBSerial.println("Stop command received: Landing");
                } else {
                    esp_now_command_received = false;  // 不正なコマンドは無視
                }
            }
        }
        
        // 安全チェック
        // 最大飛行時間超過
        if (millis() - auto_timer > MAX_FLIGHT_TIME) {
            auto_state = AUTO_LANDING;
            USBSerial.println("Auto flight: Max time reached - Landing");
        }
        // OverG検出
        if (OverG_flag == 1) {
            auto_state = AUTO_COMPLETE;
            USBSerial.println("Auto flight: OverG detected - Emergency stop");
        }
        
        if (auto_state != old_auto_state && old_auto_state == AUTO_WAIT) ahrs_reset();

        // Get command
        get_command();

        // Angle Control
        angle_control();

        // Rate Control
        rate_control();
    } else if (auto_state == AUTO_WAIT) {
        // センサーキャリブレーション待機 + コマンド待機
        static bool calibration_done = false;
        
        // センサーキャリブレーション完了チェック
        if (!calibration_done && millis() - phase_start_time > WAIT_TIME) {
            // AHRSリセット
            for (int i = 0; i < 20; i++) {
                ahrs_reset();
            }
            calibration_done = true;
            USBSerial.println("Calibration complete. Waiting for 'start' command...");
        }
        
        // ESP-NOWコマンドチェック
        if (calibration_done && esp_now_command_received) {
            if (strcmp((char*)received_command, "start") == 0) {
                esp_now_command_received = false;  // フラグリセット
                auto_state = AUTO_TAKEOFF;
                phase_start_time = millis();
                Alt_ref = TARGET_ALTITUDE;  // 目標高度50cm
                USBSerial.println("Start command received: Takeoff to 50cm");
            } else {
                esp_now_command_received = false;  // 不正なコマンドは無視
            }
        }

        // Parking
        motor_stop();
        OverG_flag = 0;
        Thrust0    = 0.0;
        Alt_flag   = 0;
        // Reset variables
        Roll_rate_reference  = 0;
        ahrs_reset_flag      = 0;  // 小文字に統一
        Range0flag           = 0;
        Alt_ref = TARGET_ALTITUDE;  // 目標高度を維持
        Landing_state        = 0;
        Auto_takeoff_counter = 0;
        Thrust_filtered.reset();
        EstimatedAltitude.reset();
        Duty_fr.reset();
        Duty_fl.reset();
        Duty_rr.reset();
        Duty_rl.reset();
    } else if (auto_state == AUTO_LANDING) {
        if (auto_landing() == 1) {
            auto_state = AUTO_COMPLETE;
            USBSerial.println("Auto flight: Landed successfully");
        }

        // Angle Control
        angle_control();

        // Rate Control
        rate_control();
    } else if (auto_state == AUTO_COMPLETE) {
        // Parking
        motor_stop();
        OverG_flag = 0;
        Thrust0    = 0.0;
        Alt_flag   = 0;
        // Reset variables
        Roll_rate_reference  = 0;
        ahrs_reset_flag      = 0;
        Range0flag           = 0;
        Alt_ref = Alt_ref0;
        Landing_state        = 0;
        Auto_takeoff_counter = 0;
        Thrust_filtered.reset();
        EstimatedAltitude.reset();
        Duty_fr.reset();
        Duty_fl.reset();
        Duty_rr.reset();
        Duty_rl.reset();
        
        // 完了メッセージ
        USBSerial.println("Flight complete. System ready for next flight.");
    }


    uint32_t ce_time = micros();
    Dt_time          = ce_time - cs_time;
    old_auto_state   = auto_state;  // Memory previous state
    // OldMode = Mode;  // Modeは使用しない
    // End of Loop_400Hz function
}



///////////////////////////////////////////////////////////////////
//  PID control gain setting
//
//  Sets the gain of PID control.
//
//  Function usage
//  PID.set_parameter(PGAIN, IGAIN, DGAIN, TC, STEP)
//
//  PGAIN: PID Proportional Gain
//  IGAIN: PID Integral Gain
//   *The larger the value of integral gain, the smaller the effect of integral control.
//  DGAIN: PID Differential Gain
//  TC:    Time constant for Differential control filter
//  STEP:  Control period
//
//  Example
//  Set roll rate control PID gain
//  p_pid.set_parameter(2.5, 10.0, 0.45, 0.01, 0.001);

void control_init(void) {
    // Rate control
    p_pid.set_parameter(Roll_rate_kp, Roll_rate_ti, Roll_rate_td, Roll_rate_eta,
                        Control_period);  // Roll rate control gain
    q_pid.set_parameter(Pitch_rate_kp, Pitch_rate_ti, Pitch_rate_td, Pitch_rate_eta,
                        Control_period);  // Pitch rate control gain
    r_pid.set_parameter(Yaw_rate_kp, Yaw_rate_ti, Yaw_rate_td, Yaw_rate_eta, Control_period);  // Yaw rate control gain

    // Angle control
    phi_pid.set_parameter(Rall_angle_kp, Rall_angle_ti, Rall_angle_td, Rall_angle_eta,
                          Control_period);  // Roll angle control gain
    theta_pid.set_parameter(Pitch_angle_kp, Pitch_angle_ti, Pitch_angle_td, Pitch_angle_eta,
                            Control_period);  // Pitch angle control gain

    // Altitude control
    alt_pid.set_parameter(alt_kp, alt_ti, alt_td, alt_eta, alt_period);
    z_dot_pid.set_parameter(z_dot_kp, z_dot_ti, z_dot_td, alt_eta, alt_period);

    Duty_fl.set_parameter(0.003, Control_period);
    Duty_fr.set_parameter(0.003, Control_period);
    Duty_rl.set_parameter(0.003, Control_period);
    Duty_rr.set_parameter(0.003, Control_period);

    Thrust_filtered.set_parameter(0.01, Control_period);
}
///////////////////////////////////////////////////////////////////

float get_trim_duty(float voltage) {
    return -0.2448f * voltage + 1.5892f;
}

// 外部変数の宣言（main.cppで定義）
extern volatile float target_roll_angle;
extern volatile float target_pitch_angle;
extern volatile bool angle_command_active;
extern volatile uint32_t last_angle_command_time;
extern volatile uint32_t target_angle_sequence;
extern bool send_angle_feedback(float roll_cmd, float pitch_cmd, uint32_t sequence);

void get_command(void) {
    // 自動飛行用に簡素化 - 高度制御は自動、姿勢は外部指令
    
    // Auto Throttle Altitude Control (高度自動制御)
    Alt_flag = 1;
    if (Auto_takeoff_counter < 500) {
        Thrust0 = (float)Auto_takeoff_counter / 1000.0;
        if (Thrust0 > get_trim_duty(3.8)) Thrust0 = get_trim_duty(3.8);
        Auto_takeoff_counter++;
    } else if (Auto_takeoff_counter < 1000) {
        Thrust0 = (float)Auto_takeoff_counter / 1000.0;
        if (Thrust0 > get_trim_duty(Voltage)) Thrust0 = get_trim_duty(Voltage);
        Auto_takeoff_counter++;
    } else {
        Thrust0 = get_trim_duty(Voltage);
    }
    
    // Alt_refは後で自動シーケンスで設定される
    // Range0flag処理は残す（安全機能）
    if ((Range0flag > OladRange0flag) || (Range0flag == RNAGE0FLAG_MAX)) {
        Thrust0        = Thrust0 - 0.02;
        OladRange0flag = Range0flag;
    }
    Thrust_command = Thrust0 * BATTERY_VOLTAGE;
    
    // 姿勢指令値
    if (angle_command_active) {
        // 外部からの角度指令を使用
        // 通信タイムアウトチェック（200ms以上古いデータは使用しない）
        if (millis() - last_angle_command_time < 200) {
            Roll_angle_command = target_roll_angle + Roll_angle_bias;
            Pitch_angle_command = target_pitch_angle + Pitch_angle_bias;
        } else {
            // タイムアウト時は水平維持
            Roll_angle_command = 0.0f + Roll_angle_bias;
            Pitch_angle_command = 0.0f + Pitch_angle_bias;
        }
    } else {
        // 角度制御非アクティブ時は水平維持
        Roll_angle_command = 0.0f + Roll_angle_bias;
        Pitch_angle_command = 0.0f + Pitch_angle_bias;
    }

    static uint32_t last_feedback_sequence = 0;
    static uint32_t last_feedback_time_ms = 0;
    uint32_t feedback_sequence = angle_command_active ? target_angle_sequence : 0;
    uint32_t now_ms = millis();

    bool should_send_feedback = false;
    if (feedback_sequence != last_feedback_sequence) {
        should_send_feedback = true;
    } else if ((now_ms - last_feedback_time_ms) >= 10) {
        should_send_feedback = true;
    }

    if (should_send_feedback) {
        float feedback_roll = Roll_angle_command;
        float feedback_pitch = Pitch_angle_command;
        if (send_angle_feedback(feedback_roll, feedback_pitch, feedback_sequence)) {
            last_feedback_sequence = feedback_sequence;
            last_feedback_time_ms = now_ms;
        }
    }

    // ヨーは常に0（回転しない）
    Yaw_angle_command = 0.0f;
    Yaw_rate_reference = 0.0f;
}


uint8_t auto_landing(void) {
    // Auto Landing - 自動飛行用に簡素化
    uint8_t flag;
    static float old_alt[10];

    Alt_flag = 0;
    if (Landing_state == 0) {
        Landing_state = 1;
        for (uint8_t i = 0; i < 10; i++) old_alt[i] = Altitude2;
        Thrust0 = get_trim_duty(Voltage);
    }
    
    // 降下速度制御
    if (old_alt[9] >= Altitude2) {
        Thrust0 = Thrust0 * 0.9999;
    }
    if (Altitude2 < 0.15) {
        Thrust0 = Thrust0 * 0.999;
    }
    if (Altitude2 < 0.1) {
        flag          = 1;
        Landing_state = 0;
    } else {
        flag = 0;
    }

    for (int i = 1; i < 10; i++) old_alt[i] = old_alt[i - 1];
    old_alt[0] = Altitude2;

    // 姿勢指令値は全て0（水平維持）
    Roll_angle_command = 0.0f;
    Pitch_angle_command = 0.0f;
    Yaw_angle_command = 0.0f;
    Yaw_rate_reference = 0.0f;

    return flag;
}

void rate_control(void) {
    float p_rate, q_rate, r_rate;
    float p_ref, q_ref, r_ref;
    float p_err, q_err, r_err, z_dot_err;

    // Rate Control
    if (Thrust_command / BATTERY_VOLTAGE < Motor_on_duty_threshold) {
        reset_rate_control();
    } else {
        // Control angle velocity
        p_rate = Roll_rate;
        q_rate = Pitch_rate;
        r_rate = Yaw_rate;

        // Get reference
        p_ref = Roll_rate_reference;
        q_ref = Pitch_rate_reference;
        r_ref = Yaw_rate_reference;

        // Error
        p_err = p_ref - p_rate;
        q_err = q_ref - q_rate;
        r_err = r_ref - r_rate;

        // Rate Control PID
        Roll_rate_command  = p_pid.update(p_err, Interval_time);
        Pitch_rate_command = q_pid.update(q_err, Interval_time);
        Yaw_rate_command   = r_pid.update(r_err, Interval_time);

        // Altutude Control
        if (Alt_flag == 1) {
            z_dot_err      = Z_dot_ref - Alt_velocity;
            Thrust_command = Thrust_filtered.update(
                (Thrust0 + z_dot_pid.update(z_dot_err, Interval_time)) * BATTERY_VOLTAGE, Interval_time);
            if (Thrust_command / BATTERY_VOLTAGE > Thrust0 * 1.15f) Thrust_command = BATTERY_VOLTAGE * Thrust0 * 1.15f;
            if (Thrust_command / BATTERY_VOLTAGE < Thrust0 * 0.85f) Thrust_command = BATTERY_VOLTAGE * Thrust0 * 0.85f;
        } else if (auto_state == AUTO_LANDING) {
            z_dot_err      = -0.15 - Alt_velocity;
            Thrust_command = Thrust_filtered.update(
                (Thrust0 + z_dot_pid.update(z_dot_err, Interval_time)) * BATTERY_VOLTAGE, Interval_time);
            // if (Thrust_command/BATTERY_VOLTAGE > Thrust0*1.1f ) Thrust_command = BATTERY_VOLTAGE*Thrust0*1.1f;
            // if (Thrust_command/BATTERY_VOLTAGE < Thrust0*0.9f ) Thrust_command = BATTERY_VOLTAGE*Thrust0*0.9f;
        }

        // Motor Control
        // 正規化Duty
        FrontRight_motor_duty = Duty_fr.update(
            (Thrust_command + (-Roll_rate_command + Pitch_rate_command + Yaw_rate_command) * 0.25f) / BATTERY_VOLTAGE,
            Interval_time);
        FrontLeft_motor_duty = Duty_fl.update(
            (Thrust_command + (Roll_rate_command + Pitch_rate_command - Yaw_rate_command) * 0.25f) / BATTERY_VOLTAGE,
            Interval_time);
        RearRight_motor_duty = Duty_rr.update(
            (Thrust_command + (-Roll_rate_command - Pitch_rate_command - Yaw_rate_command) * 0.25f) / BATTERY_VOLTAGE,
            Interval_time);
        RearLeft_motor_duty = Duty_rl.update(
            (Thrust_command + (Roll_rate_command - Pitch_rate_command + Yaw_rate_command) * 0.25f) / BATTERY_VOLTAGE,
            Interval_time);

        const float minimum_duty = 0.0f;
        const float maximum_duty = 0.95f;

        if (FrontRight_motor_duty < minimum_duty) FrontRight_motor_duty = minimum_duty;
        if (FrontRight_motor_duty > maximum_duty) FrontRight_motor_duty = maximum_duty;

        if (FrontLeft_motor_duty < minimum_duty) FrontLeft_motor_duty = minimum_duty;
        if (FrontLeft_motor_duty > maximum_duty) FrontLeft_motor_duty = maximum_duty;

        if (RearRight_motor_duty < minimum_duty) RearRight_motor_duty = minimum_duty;
        if (RearRight_motor_duty > maximum_duty) RearRight_motor_duty = maximum_duty;

        if (RearLeft_motor_duty < minimum_duty) RearLeft_motor_duty = minimum_duty;
        if (RearLeft_motor_duty > maximum_duty) RearLeft_motor_duty = maximum_duty;

        // Duty set
        if (OverG_flag == 0) {
            set_duty_fr(FrontRight_motor_duty);
            set_duty_fl(FrontLeft_motor_duty);
            set_duty_rr(RearRight_motor_duty);
            set_duty_rl(RearLeft_motor_duty);
        } else {
            FrontRight_motor_duty = 0.0;
            FrontLeft_motor_duty  = 0.0;
            RearRight_motor_duty  = 0.0;
            RearLeft_motor_duty   = 0.0;
            motor_stop();
            // OverG_flag=0;
            auto_state = AUTO_COMPLETE;
        }
    }
}

void reset_rate_control(void) {
    motor_stop();
    FrontRight_motor_duty = 0.0;
    FrontLeft_motor_duty  = 0.0;
    RearRight_motor_duty  = 0.0;
    RearLeft_motor_duty   = 0.0;
    Duty_fr.reset();
    Duty_fl.reset();
    Duty_rr.reset();
    Duty_rl.reset();
    p_pid.reset();
    q_pid.reset();
    r_pid.reset();
    alt_pid.reset();
    z_dot_pid.reset();
    Roll_rate_reference  = 0.0f;
    Pitch_rate_reference = 0.0f;
    Yaw_rate_reference   = 0.0f;
    Rudder_center        = Yaw_angle_command;
    // angle control value reset
    Roll_rate_reference  = 0.0f;
    Pitch_rate_reference = 0.0f;
    phi_pid.reset();
    theta_pid.reset();
    phi_pid.set_error(Roll_angle_reference);
    theta_pid.set_error(Pitch_angle_reference);
    Roll_angle_offset  = 0;
    Pitch_angle_offset = 0;
}

void reset_angle_control(void) {
    Roll_rate_reference  = 0.0f;
    Pitch_rate_reference = 0.0f;
    phi_pid.reset();
    theta_pid.reset();
    // Alt_ref_filter.reset();
    phi_pid.set_error(Roll_angle_reference);
    theta_pid.set_error(Pitch_angle_reference);
    // 角度制御リセット
    Aileron_center     = Roll_angle_command;
    Elevator_center    = Pitch_angle_command;
    Roll_angle_offset  = 0;
    Pitch_angle_offset = 0;
}

void angle_control(void) {
    float phi_err, theta_err, alt_err;

    // PID Control
    if (Thrust_command / BATTERY_VOLTAGE < Motor_on_duty_threshold) {
        // Initialize
        reset_angle_control();
    } else {
        // Altitude Control PID
        alt_err = Alt_ref - Altitude2;
        if (Alt_flag >= 1) Z_dot_ref = alt_pid.update(alt_err, Interval_time);

        // Angle Control (常にANGLECONTROLモード)
        // 角度指令値をそのまま使用（既にラジアン単位）
        Roll_angle_reference  = Roll_angle_command;
        Pitch_angle_reference = Pitch_angle_command;
        
        // 角度制限（±30度）
        const float MAX_ANGLE = 30.0f * PI / 180.0f;
        if (Roll_angle_reference > MAX_ANGLE) Roll_angle_reference = MAX_ANGLE;
        if (Roll_angle_reference < -MAX_ANGLE) Roll_angle_reference = -MAX_ANGLE;
        if (Pitch_angle_reference > MAX_ANGLE) Pitch_angle_reference = MAX_ANGLE;
        if (Pitch_angle_reference < -MAX_ANGLE) Pitch_angle_reference = -MAX_ANGLE;

        // Error
        phi_err   = Roll_angle_reference - (Roll_angle - Roll_angle_offset);
        theta_err = Pitch_angle_reference - (Pitch_angle - Pitch_angle_offset);

        // Angle Control PID
        Roll_rate_reference  = phi_pid.update(phi_err, Interval_time);
        Pitch_rate_reference = theta_pid.update(theta_err, Interval_time);
    }
}

void set_duty_fr(float duty) {
    ledcWrite(FrontRight_motor, (uint32_t)(255 * duty));
}
void set_duty_fl(float duty) {
    ledcWrite(FrontLeft_motor, (uint32_t)(255 * duty));
}
void set_duty_rr(float duty) {
    ledcWrite(RearRight_motor, (uint32_t)(255 * duty));
}
void set_duty_rl(float duty) {
    ledcWrite(RearLeft_motor, (uint32_t)(255 * duty));
}

void init_pwm(void) {
    ledcSetup(FrontLeft_motor, freq, resolution);
    ledcSetup(FrontRight_motor, freq, resolution);
    ledcSetup(RearLeft_motor, freq, resolution);
    ledcSetup(RearRight_motor, freq, resolution);
    ledcAttachPin(pwmFrontLeft, FrontLeft_motor);
    ledcAttachPin(pwmFrontRight, FrontRight_motor);
    ledcAttachPin(pwmRearLeft, RearLeft_motor);
    ledcAttachPin(pwmRearRight, RearRight_motor);
}

void motor_stop(void) {
    set_duty_fr(0.0);
    set_duty_fl(0.0);
    set_duty_rr(0.0);
    set_duty_rl(0.0);
}
