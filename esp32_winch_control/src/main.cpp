#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

// [TinyML 라이브러리 헤더]
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "wind_model_data.h" 

// ==========================================
// 파이썬 11개 계수
// ==========================================
const float SCALER_MEAN[11] = { 
    -0.006287f, 0.016480f, 0.074093f, -0.078672f, 
    1.042074f, 0.026497f, -0.005405f, -0.028943f, 
    0.078355f, 0.378174f, -0.000005f 
};

const float SCALER_SCALE[11] = { 
    0.074390f, 0.069916f, 0.046384f, 0.050677f, 
    0.031026f, 0.461698f, 0.418759f, 1.938547f, 
    0.067777f, 0.260075f, 0.014837f 
};

// TinyML 텐서보드 메모리 (16KB)
const int kTensorArenaSize = 16 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* tflite_input = nullptr;
TfLiteTensor* tflite_output = nullptr;

// ==========================================
// 1. 하드웨어 핀 및 기구부 상수 정의
// ==========================================
#define RXD2 16
#define TXD2 17
#define SD_CS 5

#define STS3215_SERVO_ID 1

#define WINCH_RADIUS_M 0.00475f     
#define ROPE_DIAMETER_M 0.00033f    
const float EFFECTIVE_WINCH_RADIUS = WINCH_RADIUS_M + (ROPE_DIAMETER_M / 2.0f); // 낚시줄이 윈치에 한바퀴 감겨있다 가정

const float RPM_PER_STEP_SEC = 60.0f / 4096.0f; 
const float METERS_PER_STEP = (2.0f * PI * EFFECTIVE_WINCH_RADIUS) / 4096.0f;

const float ENCODER_DIR_MULT = -1.0f; //줄을 감은 방향에 따라 엔코더 스텝수에 따른 길이 반전 가능

const float MAX_WINCH_LENGTH = 1.0f;    
const float MAX_ALLOWED_RPM = 45.0f;    
const int16_t MAX_MOTOR_CMD = 3072;     

const char* AP_SSID = "ESP32_Winch_AP";
const char* AP_PASS = "12345678";
const uint16_t TELEMETRY_PORT = 12345;
const uint16_t COMMAND_PORT = 12346;

volatile bool enable_sd_logging = false;
volatile uint8_t sd_status_global = 0; 

// ==========================================
// 2. 통신 패킷 구조체
// ==========================================
typedef struct __attribute__((packed)) { //Padding Byte 제거 후 Packing
    float roll, pitch, gyroX, gyroY, gyroZ, accX, accY, accZ;
} EspNowPayload; //ESP32C3+MPU6050 IMU로부터 ESP NOW 통신을 통해 수신 받은 자세 및 관성 데이터, 32 Byte

typedef struct __attribute__((packed)) {
    uint32_t timestamp;  //메인보드 동작시간
    float roll, pitch, gyroX, gyroY;  //영점 보정 후 박스 각도와 각속도     
    float theta_mag, length, L_dot;   // roll, pitch 합성 각 (thetamag), 줄길이 (length), 줄속도 (L_dot)   
    uint8_t smc_state;  //SMC 제어기 상태 (0: 대기/off, 1: 제어 중, 2: 미영점, 3: 비상정지)
    float tinyml_comp;  //TinyML 알고리즘이 예측해 보정한 바람 외란 값
    uint8_t sd_status;  //SD카드 기록 상태 (0: 비활성, 1: 정상 기록 중, 2: 오류)
} TelemetryPacket; //사용자의 PC GUI로 UDP 통신을 통해 송신하는 텔레메트리, 34 Byte    

typedef struct __attribute__((packed)) {
    uint8_t cmd_type; 
    float param1, param2, param3;    
} CommandPacket; //사용자의 PC GUI에서 UDP 통신을 통해 수신받은 원격 동작 지령, 13 Byte

typedef struct {
    uint32_t timestamp, dt;
    float roll, pitch, accX, accY, accZ, gyroX, gyroY, gyroZ;
    float theta_mag, length, L_dot, L_dot_cmd;
    float L_enc_meas;   //엔코더로 측정한 줄 길이   
    float L_dot_encoder; //엔코더로 측정한 줄 속도  
    int16_t motor_cmd; //모터 출력 지령
    float error, s_val; //오차, SMC 슬라이딩면 (s) 값
    uint8_t smc_state; // SMC 상태
    float tinyml_comp; //tinyml 외란 보상값
    int16_t raw_enc;   //엔코더 원시 스텝값    
} SdLogPacket; //100Hz 제어루프 정보를 SD 카드 CSV 파일로 기록

// ==========================================
// 3. 글로벌 변수 및 동기화 객체
// ==========================================
SemaphoreHandle_t dataMutex; //데이터 동시 접근 오류 방지 Mutex
SemaphoreHandle_t uartMutex; //모터 통신 동시 접근 오류 방지 Mutex
QueueHandle_t sdQueue; //100Hz 고속 데이터를 SD 카드 기록 스레드로 안전하게 전달하는 Queue

volatile float raw_roll = 0.0f, raw_pitch = 0.0f; //volatile로 실제 RAM 주소 접근, IMU의 보정 전 각도
volatile float roll_offset = 0.0f, pitch_offset = 0.0f; //영점 보정 시 저장되는 오프셋
volatile float current_gyroX = 0.0f, current_gyroY = 0.0f, current_gyroZ = 0.0f; //3축 각속도
volatile float current_accX = 0.0f, current_accY = 0.0f, current_accZ = 0.0f; //3축 가속도

volatile float L_current = 0.0f; //윈치 길이 추정치
volatile float L_dot_cmd_global = 0.0f; //모터에 내려진 속도 명령

volatile int16_t raw_encoder_pos_global = 0; //모터 엔코더에서 읽은 원시 스텝값
volatile float L_encoder_meas = 0.0f; //엔코더로부터 역산한 줄 길이
volatile float L_dot_encoder_global = 0.0f; //엔코더로부터 역산한 줄 속도

volatile bool cmd_length_zero_flag = false; //GUI에서 줄 길이 0m로 초기화하는 명령

volatile uint8_t smc_state_global = 0; //SMC 제어 상태
volatile float tinyml_compensation = 0.0f; //tinyml이 계산한 보상값
volatile float th_tolerable_deg_global = 4.0f; //제어기를 켜는 임계각, 기본 4도 (GUI에서 변경가능)

volatile bool is_zeroed = false;  //GUI에서 IMU 영점을 잡았는지 여부   
volatile bool e_stop_active = false; //GUI에서 비상정지 (E-stop) 눌렀는지 여부

char sd_log_filename[32] = "/winch_log.csv"; //SD 카드에 저장되는 csv 파일명

typedef struct {
    uint8_t mode; //0(대기), 1(수동 동작), 2(자동 서보/호버링 동작), 3(자동 감기)
    float manual_speed; //수동 제어 시 목표 속도
    uint8_t manual_override; //줄이 덜 감겼을 시 수동으로 더 감기 가능
    float auto_target_L, auto_speed, auto_hover_time; //자동 모드용 목표 길이, 속도, 호버링 대기시간
    uint8_t auto_state; // 자동모드 상태 (하강/호버링/상승)
    uint32_t hover_start_ms; //호버링 시작한 순간 타임 스템프
} SystemCommand; // 사용자 PC GUI로부터 전달받은 지령

volatile SystemCommand current_cmd = {0, 0.0f, 0, 0.5f, 0.015f, 3.0f, 0, 0}; //지령 초기치

WiFiUDP udpTelemetry; //GUI에게 UDP로 송신하는 텔레메트리 
WiFiUDP udpCommand; //GUI로부터 UDP로 수신받는 명령
IPAddress remoteIP(192, 168, 4, 2); //사용자 PC GUI 접속 주소

// ==========================================
// 4. STS3215 서보모터 드라이버
// ==========================================
void initSTS3215() { //서보 가동 함수
    delay(50);
    uint8_t buf1[8] = {0xFF, 0xFF, (uint8_t)STS3215_SERVO_ID, 4, 0x03, 0x21, 0x01, 0x00}; // buf1: 주소 0x21 (Operation Mode 레지스터)에 값 0x01 (정속 회전 모드) 쓰기
    uint8_t cs1 = 0;
    for (int i = 2; i < 7; i++) cs1 += buf1[i]; //Checksum을 자동 계산해 8번째 바이트(buf[7])에 넣고 Serial2로 전송
    buf1[7] = ~cs1;
    Serial2.write(buf1, 8);
    delay(20);

    uint8_t buf2[8] = {0xFF, 0xFF, (uint8_t)STS3215_SERVO_ID, 4, 0x03, 0x28, 0x01, 0x00}; // buf2: 주소 0x28 (Torque Switch 레지스터)에 값 0x01 (토크 인가) 쓰기
    uint8_t cs2 = 0;
    for (int i = 2; i < 7; i++) cs2 += buf2[i];
    buf2[7] = ~cs2;
    Serial2.write(buf2, 8);
    delay(20);
}

void setSTS3215Speed(int16_t speed) { //모터에 속도명령 전송 함수
    uint16_t raw_speed = 0;
    int16_t abs_spd = abs(speed);
    
    if (abs_spd > MAX_MOTOR_CMD) abs_spd = MAX_MOTOR_CMD; // 최대 속도 제한
    if (speed > 0) raw_speed = (uint16_t)abs_spd | (1 << 15); //속도가 양수이면 주소 0x2E (속도 레지스터)의 MSB (15번째 비트)에 1 (정회전) 쓰기
    else raw_speed = (uint16_t)abs_spd; //속도가 음수이면 0 (역회전) 쓰기

    uint8_t buf[9] = {0xFF, 0xFF, STS3215_SERVO_ID, 5, 0x03, 0x2E, 
                      (uint8_t)(raw_speed & 0xFF), (uint8_t)((raw_speed >> 8) & 0xFF), 0}; 
    //{Header, Header, Servo ID, 패킷길이, 0x03: Instruction (쓰기), 0x2E: Register Address (속도), Data low byte, Data high byte, Checksum}
    uint8_t checksum = 0;
    for (int i = 2; i < 8; i++) checksum += buf[i];
    buf[8] = ~checksum;

    TickType_t wait_ticks = (speed == 0 || e_stop_active) ? pdMS_TO_TICKS(50) : pdMS_TO_TICKS(5);
    //정지명령 (speed==0), 비상정지 (e_stop_active)일떄는 Mutex 대기 50ms (정지, 비상정지 명령은 50ms까지 기다려 반드시 명령 전송)
    //일반 제어시 5ms 대기 (100Hz (10ms) 제어루프 내 5ms 이상 막히면 제어 루프 안정성을 위해 다음 주기에 전송)
    if (xSemaphoreTake(uartMutex, wait_ticks) == pdTRUE) {
        Serial2.write(buf, 9);
        xSemaphoreGive(uartMutex);
    }//다른 쓰레드에서 Serial2로 동시에 접근하지 못하게 xSemaphoreTake로 Mutex 획득/반납
}

int16_t readSTS3215Position() { //모터에서 엔코더 읽는 함수
    uint8_t tx_buf[8] = {0xFF, 0xFF, STS3215_SERVO_ID, 4, 0x02, 0x38, 0x02, 0}; 
    // {Header,Header,Servo ID, 패킷길이, 0x02: READ 명령어, 0x38: Current location, 0x02: 2바이트 읽어오기}
    uint8_t cs = 0;
    for(int i = 2; i < 7; i++) cs += tx_buf[i];
    tx_buf[7] = ~cs; //Checksum

    if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(5)) == pdTRUE) { //5ms 동안 Mutex 대기
        while(Serial2.available()) Serial2.read(); //수신버퍼의 쓰레기 값 비우기
        Serial2.write(tx_buf, 8); //읽기 요청 패킷 8 Byte를 모터로 전송 
        
        uint8_t rx_buf[8]; 
        size_t rx_len = Serial2.readBytes(rx_buf, 8); //모터의 8byte 응답을 Rx 버퍼에 저장
        xSemaphoreGive(uartMutex); //수신 완료 후 Mutex 반납

        if(rx_len == 8 && rx_buf[0] == 0xFF && rx_buf[1] == 0xFF && rx_buf[4] == 0x00) { // 수신 데이터 정상인지 8Byte, 헤더, Error 바이트 검사
            uint8_t rx_cs = 0;
            for(int i = 2; i < 7; i++) rx_cs += rx_buf[i];
            rx_cs = ~rx_cs;
            if(rx_cs == rx_buf[7]) { //Checksum
                int16_t pos = rx_buf[5] | (rx_buf[6] << 8); //상위, 하위 바이트를 합쳐 16Byte 정수 스텝값으로 복원
                return pos;
            }
        }
    }
    return -1; //검증, 복원 실패시 에러
}

// ==========================================
// 5. 비동기 엔코더 읽기 타스크 (델타 누적 방식, 50Hz)
// ==========================================
void encoderTask(void *pvParameters) {
    int16_t last_raw_pos = -1; //직전 주기에 읽은 원시 엔코더 값 (초기화 여부 판단)
    int32_t current_absolute_steps = 0; //연속적으로 누적된 총 절대 스텝 수
    int32_t zero_offset_steps = 0; //줄 길이 0m의 기준이 되는 영점 스텝 수
    
    float prev_L_encoder = 0.0f; //속도 계산용 직전 주기의 줄 길이 (m)
    uint64_t prev_enc_time_us = esp_timer_get_time(); //속도 계산용 직전 주기의 측정시간 (us)

    TickType_t xLastWakeTime = xTaskGetTickCount(); //vTaskDelayUntil 전용 기준 시간 변수 초기화
    const TickType_t xFrequency = pdMS_TO_TICKS(20); //20ms 주기
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (e_stop_active) {
            vTaskDelay(pdMS_TO_TICKS(50));
            xLastWakeTime = xTaskGetTickCount();
            continue; //비상정지 플래그 검사 후 맞다면 50ms 동안 휴식 후 건너뛰기
        }

        int16_t raw_pos = readSTS3215Position(); //모터로부터 0~4095의 절대 위치값 가져오기
        if (raw_pos != -1) {
            raw_encoder_pos_global = raw_pos; //정상 수신 시 글로벌 변수에 업데이트
            
            if (last_raw_pos == -1) { //첫번째 성공적인 수신일때만 실행
                last_raw_pos = raw_pos;
                zero_offset_steps = raw_pos; 
                current_absolute_steps = raw_pos; //초기영점 기준, 절대 스텝 기준값으로 설정
            }
            
            int32_t diff = raw_pos - last_raw_pos; //Roll over (ex: 0>4095>0) 보정
            if (diff > 2048) diff -= 4096; //반시계 방향 회전 중 롤오버 (한주기에 2048스텝 이상 회전 불가)
            else if (diff < -2048) diff += 4096; //시계 방향 회전 중 롤오버 
            
            current_absolute_steps += diff; //보정된 델탁값 diff를 누적해 몇 바퀴를 돌든 제한 없이 연속된 전체 스텝 수 계산
            last_raw_pos = raw_pos;

            if (cmd_length_zero_flag) { //사용자 PC GUI에서 길이 0m 영점 잡을 시 실행
                zero_offset_steps = current_absolute_steps;
                L_encoder_meas = 0.0f;
                L_dot_encoder_global = 0.0f;
                prev_L_encoder = 0.0f;
                prev_enc_time_us = esp_timer_get_time();
                cmd_length_zero_flag = false;
            } else { //줄길이 및 줄 속도 계산
                int32_t total_steps = current_absolute_steps - zero_offset_steps; //영점으로부터 이동한 상대 스텝수
                float current_enc_L = ENCODER_DIR_MULT * (float)total_steps * METERS_PER_STEP; //방향 계수*상대 스텝*1스텝당 이동거리
                
                uint64_t now_enc_us = esp_timer_get_time();
                float dt_enc = (float)(now_enc_us - prev_enc_time_us) / 1000000.0f; //초 단위 dt: (현재시간-이전측정시간)/1e+6
                
                if (dt_enc > 0.005f) {  //작은 분모의 미분 노이즈 방지
                    float raw_L_dot = (current_enc_L - prev_L_encoder) / dt_enc; //미분: v=dL/dt
                    const float lpf_alpha = 0.2f; //1차 LPF으로 속도 지터 부드럽게 억제
                    L_dot_encoder_global = (lpf_alpha * raw_L_dot) + ((1.0f - lpf_alpha) * L_dot_encoder_global);
                    // Filtered speed=alpha*raw_L_dot+(1-alpha)*Previous_Filtered_Speed
                    prev_L_encoder = current_enc_L;
                    prev_enc_time_us = now_enc_us;
                }
                L_encoder_meas = current_enc_L; //최신 줄 길이 업데이트 (추후 상보필터로 3.7%만큼 반영됨)
            }
        }
        
    }
}

// ==========================================
// ESP-NOW 콜백
// ==========================================
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) { //EPS-NOW 수신 콜백 함수
    if (len == sizeof(EspNowPayload)) { //수신 데이터 크기가 사전 약속된 EspNowPaylaod 구조체와 일치하는지 검사
        EspNowPayload payload;
        memcpy(&payload, incomingData, sizeof(payload)); // 수신딘 원시 데이터를 내부 데이터 payload 구조체에 저장
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) { //Data race 막기 위해 Mutes 2ms 요청 
            raw_roll = payload.roll; raw_pitch = payload.pitch; //Roll, Pitch각도 
            current_gyroX = payload.gyroX; current_gyroY = payload.gyroY; current_gyroZ = payload.gyroZ; //3축 각속도
            current_accX = payload.accX; current_accY = payload.accY; current_accZ = payload.accZ; //3축 가속도
            xSemaphoreGive(dataMutex); //Mutex 반납
        }
    }
}

// ==========================================
// 6. SMC 및 모터 제어 타스크 (Core 1, 100Hz 고정)
// ==========================================
void smcControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount(); // 100Hz 고정 주기 생성:
    const TickType_t xFrequency = pdMS_TO_TICKS(10); 
    
    uint64_t last_loop_us = esp_timer_get_time();

    for (;;) {
        uint64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_loop_us) / 1000000.0f;
        uint32_t current_dt_ms = (uint32_t)((now_us - last_loop_us) / 1000);
        last_loop_us = now_us; //실제 dt 기록, 
        
        if (dt <= 0.001f || dt > 0.050f) dt = 0.010f; // dt 1ms 이하, 50ms 초과시 10ms로 간주


        if (!is_zeroed || e_stop_active) { //영점 미입력 또는 비상정지 일시 모터 멈추고 루프 건너뜀
            setSTS3215Speed(0);
            L_dot_cmd_global = 0.0f;
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue; 
        }

        float th_tol = th_tolerable_deg_global; //사용자가 PC GUI에 입력하는 SMC 제어각
        if (th_tol < 1.0f) th_tol = 1.0f; 
        float TH_UPPER = (th_tol * 1.5f) * (PI / 180.0f); // 제어기 발동 조건은 입력각도의 1.5배 (ex 4도 입력시 6도 초과시 발동)
        float TH_LOWER = (th_tol * 0.5f) * (PI / 180.0f); //제어기 종료 조건은 입력각도의 0.5배 (ex 4도 입력시 2도 미만시 종료)

        float r = 0.0f, p = 0.0f, gx = 0.0f, gy = 0.0f, gz = 0.0f;
        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        
        if (xSemaphoreTake(dataMutex, 0) == pdTRUE) { //Mutex을 통해 IMU 데이터 복사
            r = raw_roll - roll_offset; p = raw_pitch - pitch_offset;
            gx = current_gyroX; gy = current_gyroY; gz = current_gyroZ;
            ax = current_accX; ay = current_accY; az = current_accZ;
            xSemaphoreGive(dataMutex);
        }

        float theta_mag = sqrtf(r * r + p * p); //흔들림 크기: Roll과 Pitch 벡터의 합
        float control_error = theta_mag; 

       
        float gx_rad = gx * (PI / 180.0f); //deg/s -> rad/s
        float gy_rad = gy * (PI / 180.0f);
        float theta_dot = (theta_mag > 0.001f) ? ((r * gx_rad + p * gy_rad) / theta_mag) : 0.0f; //0으로 나누는 것 방지
        // theta_dot=angular velocity vector⋅unit axis vector
        float c_smc = 3.75f;
        float s_val = theta_dot + c_smc * theta_mag; //Sliding Surface

        if (smc_state_global == 0 && theta_mag >= TH_UPPER) smc_state_global = 1; 
        else if (smc_state_global == 1 && theta_mag <= TH_LOWER) smc_state_global = 0; //Hysteresis Control (채터링 방지)

        float L_temp = L_current; //추정 줄 길이를 임시 변수에 복사
        float L_dot_cmd = 0.0f;
        float target_L_ref = L_temp; //목표 길이를 현재 줄 길이로 설졍

        if (current_cmd.mode == 0) { //모드0: 정지
            L_dot_cmd = 0.0f;
            target_L_ref = L_temp;
        } 
        else if (current_cmd.mode == 3) { //모드3: 강제 회수, 0m로 감아올림
            target_L_ref = 0.0f;
            if (L_temp > 0.0f) {  // 줄 길이 0m 보다 크다면 1.5cm/s로 끌어올림
                L_dot_cmd = -0.015f; 
            } else {
                L_dot_cmd = 0.0f;
                current_cmd.mode = 0; //동작 완료 후 속도 0, 모드 0 (정지)로 전환
            }
        } 
        else if (current_cmd.mode == 1) { //모드1: GUI에서 인가된 속도로 수동으로 올리기, 내리기 버튼을 누름
            L_dot_cmd = current_cmd.manual_speed; 
            target_L_ref = L_temp;
        } 
        else if (current_cmd.mode == 2) { //모드2: GUi에서 길이, 속도, 호버링 시간만 입력하고 자동으로 시퀀스 시작
            if (current_cmd.auto_state == 0) { //state0: 오차가 3mm이내가 될 때까지 목표 길이로 하강
                target_L_ref = fminf(current_cmd.auto_target_L, MAX_WINCH_LENGTH); //목표 길이를 사전 입력한 한계치 (1m)로 제한
                float error_L = target_L_ref - L_temp;
                if (fabsf(error_L) > 0.003f) { //목표까지 오차가 3mm이상 남았으면 자동 이동속도로 이동
                    float dir = (error_L > 0.0f) ? 1.0f : -1.0f;
                    L_dot_cmd = dir * fabsf(current_cmd.auto_speed);
                } else { //3mm 이내로 들어오면 속도0, state1 (호버링)으로 넘어감
                    L_dot_cmd = 0.0f;
                    current_cmd.auto_state = 1; 
                    current_cmd.hover_start_ms = millis();
                }
            } 
            else if (current_cmd.auto_state == 1) { //sate1: 길이 도달 후 호버링
                target_L_ref = fminf(current_cmd.auto_target_L, MAX_WINCH_LENGTH);
                float error_L = target_L_ref - L_temp;
                L_dot_cmd = constrain(error_L * 0.5f, -fabsf(current_cmd.auto_speed), fabsf(current_cmd.auto_speed));
                //P제어기: u=Kp*e , 오차에 0.5를 곱해 천천히 수렴하도록 하고 constrain으로 P제어가 자동 속도를 초과하지 않도록 제한
                if (millis() - current_cmd.hover_start_ms >= (uint32_t)(current_cmd.auto_hover_time * 1000.0f)) {
                    current_cmd.auto_state = 2; //호버링 완료 후 state2 (복귀) 명령 전환
                }
            }
            else if (current_cmd.auto_state == 2) { //state2: 다시 0m로 향해 상승
                target_L_ref = 0.0f;
                if (L_temp > 0.0f) {  //1cm 시작 해제, 0으로 변경
                    L_dot_cmd = -fabsf(current_cmd.auto_speed); 
                } else { 
                    L_dot_cmd = 0.0f;
                    current_cmd.mode = 0; 
                }
            }
        }

        // =========================================================================
        // 하이브리드 제어부
        // =========================================================================
        if (smc_state_global == 1) {
            
            float L_dot_eq = L_dot_cmd;

            if (current_cmd.mode != 0) {
                const float alpha_tinyml = 0.00075f; // 외란 tinyml 보상
                float u_eq_wind = alpha_tinyml * tinyml_compensation;

                
                L_dot_eq += u_eq_wind;
            }

            //SMC 스위칭 및 감쇄 제어량 연산
            float phi_smc = 0.05f; //Boundary Layer 두께
            float K_smc = 0.004f; //이득
            float sat_s = fmaxf(-1.0f, fminf(1.0f, s_val / phi_smc)); //포화함수 (Saturation Fuction)
            
            float damping_val = fabsf(theta_dot) - c_smc * theta_mag;
            float sign_damping = (damping_val > 0.0f) ? 1.0f : ((damping_val < 0.0f) ? -1.0f : 0.0f);
            //damping_val>0: 진자가 중심을 빠르게 지나는 상태, 에너지를 흡수하기 위해 줄을 (더) 풀어주는 방향
            //damping_val<0: 진자가 최고점에 다다르며 느리게 지나는 상태, 에너지를 흡수하기 위해 줄을 (더) 감아주는 방향
            float u_smc = K_smc * sign_damping * fabsf(sat_s);
            u_smc = constrain(u_smc, -0.005f, 0.005f); //클램핑

            
            float u_restore = 0.0f; //호버링 중에만 위치 유지 보상
            bool is_hovering = (current_cmd.mode == 0) || 
                               (current_cmd.mode == 2 && current_cmd.auto_state == 1);
            if (is_hovering) {
                float K_pos_restore = 0.08f;
                u_restore = -K_pos_restore * (L_temp - target_L_ref); //P제어기로 목표 위치로 당겨오는 복원 속도
            }

            float smc_total = u_smc + u_restore;

    
            L_dot_cmd = L_dot_eq + smc_total;  // 최종 제어 속도 명령 결합
        }

        if (current_cmd.manual_override == 0 && L_temp <= 0.00f && L_dot_cmd < 0.0f) L_dot_cmd = 0.0f; //줄 길이가 0m인데 감아 올리라는 명령 들어오면 속도 차단
        if (current_cmd.manual_override == 0 && L_temp >= MAX_WINCH_LENGTH && L_dot_cmd > 0.0f) L_dot_cmd = 0.0f; //줄 길이가 최대 인데 풀라는 명령 들어오면 속도 차단

        float omega_rad = L_dot_cmd / EFFECTIVE_WINCH_RADIUS; //v=rw로 선속도 지령을 모터 RPM 및 원시 제어값으로 변환
        float rpm = (omega_rad * 60.0f) / (2.0f * PI); //rad/s를 RPM으로 변환
        rpm = constrain(rpm, -MAX_ALLOWED_RPM, MAX_ALLOWED_RPM); //최대 허용 RPM으로 클램핑
        
        int16_t motor_cmd_val = (int16_t)(rpm / RPM_PER_STEP_SEC);
        motor_cmd_val = constrain(motor_cmd_val, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);

        float actual_rpm = (float)motor_cmd_val * RPM_PER_STEP_SEC; //정수형으로 변환된 모터 명령 대신 RPM에서 재계산해 적분에 활용
        float actual_omega = (actual_rpm * 2.0f * PI) / 60.0f;
        float actual_L_dot = actual_omega * EFFECTIVE_WINCH_RADIUS;

        L_temp += actual_L_dot * dt; //1. 내부 지령으로 역산한 길이 (Dead Reckoning) 
        
        const float ALPHA_ENCODER = 0.037f; 
        float current_L_encoder = L_encoder_meas; //2. 엔코더 기반 역산한 길이
        float encoder_error = current_L_encoder - L_temp; //Dead Reckoning와 엔코더 기반 값의 오차에 가중치 곱함
        L_temp += ALPHA_ENCODER * encoder_error; // 최종 측정길이: Dead Reckoning 96.3%, 엔코더 기반 측정 3.7%
        //내부 지령 역산 길이의 저주파 노이즈 (적분 오차)와 엔코더 기반 길이의 고주파 노이즈 (슬립, 통신지연)를 상보필터 (Complementary Filter)로 융합

        if (current_cmd.manual_override == 0) { ///클램핑 
            if (L_temp < 0.000f) L_temp = 0.000f;
            if (L_temp > MAX_WINCH_LENGTH) L_temp = MAX_WINCH_LENGTH;
        } else {
            if (L_temp < -MAX_WINCH_LENGTH) L_temp = -MAX_WINCH_LENGTH;
            if (L_temp > MAX_WINCH_LENGTH) L_temp = MAX_WINCH_LENGTH;
        }
        
        L_current = L_temp; //전역변수에 업데이트
        L_dot_cmd_global = actual_L_dot; 

        setSTS3215Speed(motor_cmd_val); //모터에 속도 명령 전송

        if (enable_sd_logging) {
            SdLogPacket sdpkt;
            sdpkt.timestamp = (uint32_t)(now_us / 1000); //SD카드에 저장할 데이터 22개
            sdpkt.dt = current_dt_ms; 
            sdpkt.roll = r; sdpkt.pitch = p;
            sdpkt.accX = ax; sdpkt.accY = ay; sdpkt.accZ = az;
            sdpkt.gyroX = gx; sdpkt.gyroY = gy; sdpkt.gyroZ = gz;
            sdpkt.theta_mag = theta_mag;
            sdpkt.length = L_current; 
            sdpkt.L_dot = actual_L_dot;
            sdpkt.L_dot_cmd = L_dot_cmd;
            
            sdpkt.L_enc_meas = current_L_encoder; 
            sdpkt.L_dot_encoder = L_dot_encoder_global; 
            sdpkt.raw_enc = raw_encoder_pos_global;

            sdpkt.motor_cmd = motor_cmd_val;
            sdpkt.error = control_error;
            sdpkt.s_val = s_val;
            sdpkt.smc_state = smc_state_global; 
            sdpkt.tinyml_comp = tinyml_compensation;
            
            xQueueSend(sdQueue, &sdpkt, 0); //SD카드 Queue에 데이터 전달
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency); //다음 10ms 주기까지 대기
    }
}

// ==========================================
// 7. 네트워크 타스크 (명령 처리 & 텔레메트리)
// ==========================================
//7.1 초기화 및 태스크 루프 설정
//---------------------------------------------
void networkTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount(); // 주기적 실행을 위한 현재 RTOS 타임스탬프 저장
    const TickType_t xFrequency = pdMS_TO_TICKS(10); //Task 실행 주기 10ms (100Hz)

    uint8_t telemetry_prescaler = 0; //텔레메트리 전송 주기를 맞추기 위한 분주비
//7.2 UDP 패킷 수신 및 패킷 검증
//---------------------------------------------------
    for (;;) {
        int packetSize;
        while ((packetSize = udpCommand.parsePacket()) > 0) { //UDP 소켓에서 수신한 패킷의 크기를 읽어서 데이터가 있다면 반복
            if (packetSize >= sizeof(CommandPacket)) { //패킷 크기가 약속한 구조체 크기 이상인지 확인
                CommandPacket cmdPkt; //수신 데이터를 담을 구조체 선언
                udpCommand.read((char*)&cmdPkt, sizeof(CommandPacket)); //버퍼에서 구조체로 메모리 복사
                remoteIP = udpCommand.remoteIP(); //답장을 보내기 위해 사용자 PC GUI IP 주소 저장
                //7.3 커스텀 패킷 처리
                //-----------------------
                if (cmdPkt.cmd_type == 1) { //명령1: 영점 재정령, 시스템 리셋
                    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) { //Mutex 요청
                        roll_offset = raw_roll; pitch_offset = raw_pitch; //현재 IMU 각도를 오프셋으로 지정
                        xSemaphoreGive(dataMutex); //Mutex 반환
                    }
                    is_zeroed = true;     //영점 설정완료 플래그   
                    e_stop_active = false;//비상정지 상태 해제  
                    current_cmd.mode = 0; //정지 모드로 변경
                    L_current = 0.0f;     //추정 줄 길이를 0m로 초기화        
                    cmd_length_zero_flag = true; //줄길이 0m 초기화 플래그
                } else if (cmdPkt.cmd_type == 2) { //명령 2: 강제회수
                    current_cmd.mode = 3; //모드 3 (강제회수)로 전환
                } else if (cmdPkt.cmd_type == 3) { //명령 3: 수동조종
                    current_cmd.mode = 1; //모드1 (수동)로 전환
                    current_cmd.manual_speed = cmdPkt.param1; //Param1을 수동 속도로 지정
                    current_cmd.manual_override = (cmdPkt.param2 > 0.5f) ? 1 : 0; //Param2가 0.5 초과시 리밋 해제 Overide
                } else if (cmdPkt.cmd_type == 4) { //명령4: 자동 시퀀스 모드
                    current_cmd.mode = 2; //모드2 (자동)로 전환
                    current_cmd.auto_target_L = fminf(cmdPkt.param1, MAX_WINCH_LENGTH); //목표 길이를 최대 길이 한계로 고정
                    current_cmd.auto_speed = cmdPkt.param2; //자동 속도 설정
                    current_cmd.auto_hover_time = cmdPkt.param3; //호버링 대기 시간
                    current_cmd.auto_state = 0; //자동 시퀀스 state0부터 시작
                } else if (cmdPkt.cmd_type == 5) { //명령5: 비상정지 (E-stop)
                    e_stop_active = true; //비상정지 플래그 설정)
                    current_cmd.mode = 0; //정지 모드로 전환
                    setSTS3215Speed(0);   //서보 정지
                } else if (cmdPkt.cmd_type == 6) { //명령6: 일반 정지
                    current_cmd.mode = 0;  //정지 모드로 전환
                } else if (cmdPkt.cmd_type == 7) { //명령7: SMC 허용각 수정
                    th_tolerable_deg_global = cmdPkt.param1;
                } else if (cmdPkt.cmd_type == 8) { //명령8: 줄 길이를 0으로 영점
                    L_current = 0.0f;
                    cmd_length_zero_flag = true; 
                } else if (cmdPkt.cmd_type == 9) { //명령9: SD 카드 로깅 및 파일명 지정
                    enable_sd_logging = (cmdPkt.param1 > 0.5f); 
                    if (enable_sd_logging) {
                        snprintf(sd_log_filename, sizeof(sd_log_filename), "/winch_log_%06d%04d.csv", (int)cmdPkt.param2, (int)cmdPkt.param3); //파일명
                    }
                }
            } else {
                udpCommand.flush(); //패킷 크기가 비정상일 경우 버퍼 비움
            }
        }
        //7.4 Telemetry 패킷 생성 및 전송
        //-----------------------------
        telemetry_prescaler++; 
        if (telemetry_prescaler >= 4) {  //텔레메트리 전송 25Hz
            telemetry_prescaler = 0;

            TelemetryPacket pkt; // 전송할 텔레메트리 패킷 생성
            if (xSemaphoreTake(dataMutex, 0) == pdTRUE) { //Mutex 요청
                pkt.timestamp = millis();
                pkt.roll = raw_roll - roll_offset;    // 보정된 Roll 각도
                pkt.pitch = raw_pitch - pitch_offset; // 보정된 Pitch 각도
                pkt.gyroX = current_gyroX;
                pkt.gyroY = current_gyroY;
                pkt.theta_mag = sqrtf(pkt.roll * pkt.roll + pkt.pitch * pkt.pitch);
                pkt.length = L_current;      // 현재 추정 줄 길이
                pkt.L_dot = L_dot_cmd_global;// 현재 명령 출력 속도
                
                if (e_stop_active) pkt.smc_state = 3;   // 상태 3: 비상 정지
                else if (!is_zeroed) pkt.smc_state = 2; // 상태 2: 영점 안 잡힘
                else pkt.smc_state = smc_state_global;  // 상태 0 또는 1: SMC On/Off

                pkt.tinyml_comp = tinyml_compensation;  // TinyML 보상값
                pkt.sd_status = sd_status_global;       // SD 카드 상태
                xSemaphoreGive(dataMutex); // Mutex 반환

                udpTelemetry.beginPacket(remoteIP, TELEMETRY_PORT); // GUI IP와 포트로 UDP 패킷 전송 시작
                udpTelemetry.write((uint8_t*)&pkt, sizeof(TelemetryPacket)); // 데이터 기록
                udpTelemetry.endPacket(); // 전송 마감
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // 10ms 주기 유지
    }
}
//7.5 SD 카드 마운트 및 파일 열기
//-------------------------------
void sdLoggerTask(void *pvParameters) {
    File logFile;
    bool file_is_open = false;
    SdLogPacket sdpkt;
    
    uint8_t last_mode = 255;
    float last_smc_deg = -1.0f;
    float last_auto_spd = -1.0f;
    uint8_t flush_counter = 0;
    uint32_t last_mount_retry_ms = 0;
    
    if (!SD.begin(SD_CS)) {  // SD카드 마운트 실패 시
        Serial.println("SYS_MSG: Initial SD Mount Failed!");
        sd_status_global = 2; // SD 에러(2)
    }

    for (;;) {
        if (enable_sd_logging && !file_is_open) { // 파일이 열려있지 않을 때
            if (millis() - last_mount_retry_ms > 1000) { // 1초 간격으로 SD카드 재마운트 시도
                last_mount_retry_ms = millis();
                SD.end(); SD.begin(SD_CS);
            }
            logFile = SD.open(sd_log_filename, FILE_APPEND); // 파일 열기 (이어서 쓰기 모드)
            if (logFile) {
                file_is_open = true;
                sd_status_global = 1; // SD 상태: 정상 작동중(1)
                if (logFile.size() == 0) { // 새 파일인 경우 헤더 작성
                    logFile.println("Time,dt,Roll,Pitch,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,ThetaMag,Length,L_dot,L_dot_cmd,L_enc_meas,L_dot_encoder,MotorCmd,Error,S_val,SMCState,TinyMLComp,RawEnc");
                }
                last_mode = 255; 
            } else { // 파일 열기 실패 처리
                file_is_open = false;
                enable_sd_logging = false;
                sd_status_global = 2;
                xQueueReset(sdQueue); //Queue 대기 중이던 데이터 제거
            }
        }
        //7.6 Queue 수신 및 CSV 파일 쓰기
        //-----------------------------
        if (xQueueReceive(sdQueue, &sdpkt, pdMS_TO_TICKS(100)) == pdTRUE) { // sdQueue에서 100ms 이내에 데이터 들어오면 꺼냄
            if (enable_sd_logging && file_is_open) {
                if (last_mode != current_cmd.mode || last_smc_deg != th_tolerable_deg_global || last_auto_spd != current_cmd.auto_speed) {
                    logFile.printf("# CONFIG: Mode=%d, SMC_Deg=%.1f, Auto_L=%.3f, Auto_Spd=%.3f, Hover=%.1f\n",      // 설정 변경 시 주석 형태(# CONFIG)로 CSV 중간에 한 줄 기록
                                   current_cmd.mode, th_tolerable_deg_global, 
                                   current_cmd.auto_target_L, current_cmd.auto_speed, current_cmd.auto_hover_time);
                    last_mode = current_cmd.mode; last_smc_deg = th_tolerable_deg_global; last_auto_spd = current_cmd.auto_speed;
                }
                // CSV 데이터 행 작성
                size_t written = logFile.printf("%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%d,%.3f,%.3f,%u,%.3f,%d\n",
                    sdpkt.timestamp, sdpkt.dt, sdpkt.roll, sdpkt.pitch, sdpkt.accX, sdpkt.accY, sdpkt.accZ, sdpkt.gyroX, sdpkt.gyroY, sdpkt.gyroZ, 
                    sdpkt.theta_mag, sdpkt.length, sdpkt.L_dot, sdpkt.L_dot_cmd, 
                    sdpkt.L_enc_meas, sdpkt.L_dot_encoder, 
                    sdpkt.motor_cmd, sdpkt.error, sdpkt.s_val, sdpkt.smc_state, sdpkt.tinyml_comp,
                    sdpkt.raw_enc); 

                if (written == 0) { // 기록 실패 시 파일 닫고 에러 처리
                    logFile.close(); file_is_open = false; enable_sd_logging = false;
                    sd_status_global = 2; xQueueReset(sdQueue);
                } else if (++flush_counter >= 20) { // 데이터 20번 쓸 때마다 파일 Flush
                    logFile.flush(); flush_counter = 0;
                }
            }
        }
        
        if (!enable_sd_logging && file_is_open) { // 저장 완료 명령 들어오면 파일 안전하게 닫기
            logFile.close(); file_is_open = false; sd_status_global = 0;
        }
    }
}
//7.7 TFLite 모델 및 Interpreter 초기화
//-------------------------------------------
void tinymlTask(void *pvParameters) {
    const tflite::Model* model = tflite::GetModel(g_wind_model_data); // C 헤더에 정렬된 모델 배열을 로드
    
    static tflite::AllOpsResolver resolver; // 모델에 쓰인 TFLite 연산자(Op) 등록기
    static tflite::MicroErrorReporter micro_error_reporter;

    static tflite::MicroInterpreter static_interpreter( // TFLite 인터프리터 생성
        model, 
        resolver, 
        tensor_arena, 
        kTensorArenaSize,
        &micro_error_reporter
    );
    
    interpreter = &static_interpreter;
    
    if (interpreter->AllocateTensors() != kTfLiteOk) { //Tensor 메모리 할당
        Serial.println("TFLite AllocateTensors() Failed!");
        vTaskDelete(NULL); // 실패 시 태스크 삭제
    }
    
    tflite_input = interpreter->input(0); // 입출력 포인터 연결
    tflite_output = interpreter->output(0);

    //7.8 입력 데이터 스케일링, 정수 양자화(INT8), 추론
    //--------------------------------------------
    TickType_t xLastWakeTime = xTaskGetTickCount(); // 주기 기준점
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms 주기 (50Hz)

    for (;;) {
        if (is_zeroed && !e_stop_active) { // 영점 완료, 비상정지 아닐 때만 추론 실행
            float in_features[11];
            
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) { // 데이터 획득
                in_features[0] = raw_roll - roll_offset;
                in_features[1] = raw_pitch - pitch_offset;
                in_features[2] = current_accX;
                in_features[3] = current_accY;
                in_features[4] = current_accZ;
                in_features[5] = current_gyroX;
                in_features[6] = current_gyroY;
                in_features[7] = current_gyroZ;
                in_features[8] = sqrtf(in_features[0] * in_features[0] + in_features[1] * in_features[1]); // theta_mag
                in_features[9] = L_current;
                in_features[10] = L_dot_cmd_global;
                xSemaphoreGive(dataMutex);
            } else {
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }

            // 11개 입력 Feature에 대한 Standard Scaling 및 INT8 Quantization 적용
            for (int i = 0; i < 11; i++) {
                float scaled = (in_features[i] - SCALER_MEAN[i]) / SCALER_SCALE[i]; // (x - Mean) / Std
                float quant_float = (scaled / tflite_input->params.scale) + tflite_input->params.zero_point; // INT8 변환
                int8_t quant_val = (int8_t)fmaxf(-128.0f, fminf(127.0f, quant_float)); // [-128, 127] 범위 클램핑
                tflite_input->data.int8[i] = quant_val;
            }

            interpreter->Invoke(); //모델 추론 실행

            int8_t raw_out = tflite_output->data.int8[0]; // INT8 출력을 다시 실수(float) 단위로 Dequantize
            float pred_comp = (raw_out - tflite_output->params.zero_point) * tflite_output->params.scale;

            tinyml_compensation = pred_comp; // 추론 결과 할당
        } else {
            tinyml_compensation = 0.0f;
            xLastWakeTime = xTaskGetTickCount();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

//7.9 리얼, 하드웨어, 통신 및 멀티태스킹 멀티코어 할당
//--------------------------------------------
void setup() {
    Serial.begin(115200); // 디버깅 시리얼 시작

    Wire.begin(); Wire.setTimeOut(10); // I2C 통신 시작 및 타임아웃 설정
    
    Serial2.begin(1000000, SERIAL_8N1, RXD2, TXD2); // 서보 모터용 UART2 (1Mbps 통신)
    Serial2.setTimeout(5); 

    uartMutex = xSemaphoreCreateMutex(); // 모터 UART 보호용 Mutex
    dataMutex = xSemaphoreCreateMutex(); // 센서, 상태 데이터 보호용 Mutex
    sdQueue = xQueueCreate(100, sizeof(SdLogPacket)); // SD 로그 패킷 Queue 슬롯 100개 생성

    initSTS3215(); // STS3215 서보 모터 초기화 레지스터 세팅

    WiFi.softAP(AP_SSID, AP_PASS); // ESP32 Wi-Fi AP 핫스팟 개설
    WiFi.setTxPower(WIFI_POWER_19_5dBm); //최대 출력 19.5dBm 설정
    
    udpTelemetry.begin(TELEMETRY_PORT);  // 텔레메트리 포트 오픈
    udpCommand.begin(COMMAND_PORT);      // 명령 수신 포트 오픈
    WiFi.mode(WIFI_AP_STA);              // AP+STA 모드 설정 (ESP-NOW 병행)
    if (esp_now_init() == ESP_OK) esp_now_register_recv_cb(OnDataRecv); // ESP-NOW 콜백 함수 등록

    // FreeRTOS 태스크 생성 및 코어 할당 (Dual Core 활용)
    // xTaskCreatePinnedToCore(함수, 이름, 스택크기, 파라미터, 우선순위, 핸들, 코어ID)
    xTaskCreatePinnedToCore(sdLoggerTask, "SDTask", 3072, NULL, 1, NULL, 0); //제어 (최우선), 엔코더 (차우선)  : Core1 
    xTaskCreatePinnedToCore(smcControlTask, "SMCTask", 4096, NULL, 3, NULL, 1); //SD카드 (차차우선), 통신 (차우선), tinyml (차차우선): Core0
    xTaskCreatePinnedToCore(encoderTask, "EncTask", 2048, NULL, 2, NULL, 1);    
    xTaskCreatePinnedToCore(networkTask, "NetTask", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(tinymlTask, "MLTask", 4096, NULL, 1, NULL, 0);
}   
//7.10 loop() 파괴
//---------------
void loop() { vTaskDelete(NULL); } //FreeRTOS Task 구동으로 loop() 필요없음