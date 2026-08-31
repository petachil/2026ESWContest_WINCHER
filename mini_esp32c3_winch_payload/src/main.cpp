#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>

// ==========================================================
// 1. 센서 및 통신 정의
// ==========================================================
#define MPU6050_ADDR         0x68 //MPU6050의 기본 I2C Slave 주소
#define MPU6050_CONFIG       0x1A //MPU6050의 Digital LPF 및 외부 동기화 
#define MPU6050_ACCEL_CONFIG 0x1C //MPU6050 가속도계의 측정범위 
#define MPU6050_PWR_MGMT_1   0x6B //MPU6050 센서 슬립모드 해제, CLK 소스
#define MPU6050_ACCEL_XOUT_H 0x3B //읽기 시작할 첫번째 데이터 (X축 가속도 상위 Byte)

#define I2C_SDA 6 // ESP32-C3 I2C 데이터 핀
#define I2C_SCL 7 // ESP32-C3 I2C CLK 핀

uint8_t receiverAddress[] = {0x8C, 0x94, 0xDF, 0x6D, 0xE8, 0x30}; //수신기(메인 ESP32)의 MAC 주소

typedef struct __attribute__((packed)) { //데이터 패킷 전송용 구조체 Packed
    float roll;     // rad 단위
    float pitch;    
    float gyroX;    // rad/s 단위
    float gyroY;    
    float gyroZ;    
    float accX;     // G 단위 
    float accY;     
    float accZ;     
} struct_message; //32Byte

struct_message payloadData; //ESP-now 전송시 데이터를 담을 구조체 선언
esp_now_peer_info_t peerInfo; //수신기 (EPS32) 정보를 담을 구조체 선언

// ==========================================================
// 2. Mahony 필터 알고리즘
// ==========================================================
#define KP_DEFAULT 0.8f    
#define KI_DEFAULT 0.0f  
#define ACCEL_G_LOWER 0.6f //신뢰할 최소 가속도 (0.6G)
#define ACCEL_G_UPPER 1.4f //신뢰할 최대 가속도 (1.4G)

float gx_offset = 0.0f, gy_offset = 0.0f, gz_offset = 0.0f; //영점으로 측정한 자이로 오프셋

class AdaptiveMahony { //Mahony 필터 연산 클래스
private: //내부 변수 
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; //Quaternion (쿼터니온) 변수
public: 
    float roll_rad = 0.0f, pitch_rad = 0.0f; //최종 계산된 Roll, Pitch 각도 저장 변수
    float gyroX_rads = 0.0f, gyroY_rads = 0.0f, gyroZ_rads = 0.0f; // 오프셋 보정 후 자이로 속도 저장 변수

    void update(float ax, float ay, float az, float gx, float gy, float gz, float dt) { //센서 데이터와 시간간격을 받아 자세를 갱신하는 함수
        gyroX_rads = gx; //보정된 3축 자이로 값 저장
        gyroY_rads = gy;
        gyroZ_rads = gz; 

        float accel_norm = sqrtf(ax * ax + ay * ay + az * az); //측정한 전체 가속도 벡터의 크기
        float current_kp = KP_DEFAULT;
        if (accel_norm < ACCEL_G_LOWER || accel_norm > ACCEL_G_UPPER) current_kp = 0.0f; //충격으로 0.6G~1.4G를 벗어나면 연산 제외

        if (accel_norm > 0.0f) { //가속도 데이터가 존재할 때 연산
            ax /= accel_norm; ay /= accel_norm; az /= accel_norm; //크기가 1인 단위 벡터로 정규화
            float vx = 2.0f * (q1 * q3 - q0 * q2); //q·[0,0,1]^T ·q^-1 (센서 기준으로 본 지구의 수직 방향)
            float vy = 2.0f * (q0 * q1 + q2 * q3);
            float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
            float ex = (ay * vz - az * vy); //실제 가속도 벡터 (a)와 추정 중력 벡터 (v)의 오차 (외적)
            float ey = (az * vx - ax * vz);
            float ez = (ax * vy - ay * vx);

            gx += current_kp * ex; gy += current_kp * ey; gz += current_kp * ez; //오차에 비례제어 Kp를 곱해 Drift 보정
        }
        gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt); //쿼터니언 미분 방정식(0.5 * q * w) 및 시간 적분(dt)을 위한 사전 계산

        float qa = q0, qb = q1, qc = q2; //연산 중 백업
        q0 += (-qb * gx - qc * gy - q3 * gz); // dq/dt = 0.5 * q * w 오일러 적분 수행: 다음 자세 계산
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);

        float q_norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);  //쿼터니온 크기 계산
        if (q_norm > 0.0f) { q0 /= q_norm; q1 /= q_norm; q2 /= q_norm; q3 /= q_norm; } //소수점 연산 오차가 누적된 쿼터니온을 다시 1로 맞춰주는 정규화

        roll_rad  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)); //쿼터니온을 오일러 Roll각으로 계산, atan2로 Quadrant 모두 판별
        float sinp = 2.0f * (q0 * q2 - q3 * q1); //Pitch 각 계산을 위한 Sine 성분 추출
        if (fabsf(sinp) >= 1.0f) pitch_rad = copysignf(1.5707963f, sinp); //Gimbal Lock 발생 시 NaN 방지를 위한 pi/2로 출력
        else pitch_rad = asinf(sinp); //정상 범위 일시 오일러 Pitch 각 계산
    }
};

AdaptiveMahony filter; //Mahony 필터 객체 생성

// ==========================================================
// 3. MPU6050 센서 제어 함수
// ==========================================================
void readMPU6050(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) { //6축 센서 데이터를 읽어와 참조 변수에 전달하는 함수
    Wire.beginTransmission(MPU6050_ADDR); //I2C 통신 시작, 기 선언한 센서주소 (0x68) 지정
    Wire.write(MPU6050_ACCEL_XOUT_H); //데이터 읽기를 시작할 기 선언한 첫 번째 레지스터 주소 (0x3B, x축 가속도 상위바이트)
    Wire.endTransmission(false); //재시작 조건으로 제어권 유지
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14, true); //연속된 레지스터 데이터 14Byte 요청

    int16_t rawAx = Wire.read() << 8 | Wire.read(); //상하위 바이트 shift 연산 후 부호 있는 16비트 정수 형태의 raw 3축 가속도 합성
    int16_t rawAy = Wire.read() << 8 | Wire.read();
    int16_t rawAz = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); //가속도와 자이로 사이의 온도 센서 버림
    int16_t rawGx = Wire.read() << 8 | Wire.read(); //raw 3축 자이로 합성
    int16_t rawGy = Wire.read() << 8 | Wire.read();
    int16_t rawGz = Wire.read() << 8 | Wire.read();

    ax = (float)rawAx / 8192.0f; //+-4G (8192LSB/G)으로 나누어 3축 가속도를 G 단위 실수로 변환
    ay = (float)rawAy / 8192.0f; 
    az = (float)rawAz / 8192.0f;
    gx = ((float)rawGx / 65.5f) * DEG_TO_RAD; //+-500deg/s 감도 (65.5LSB/(deg/s))로 나누어 3축 자이로를 rad/s 단위로 변환
    gy = ((float)rawGy / 65.5f) * DEG_TO_RAD; 
    gz = ((float)rawGz / 65.5f) * DEG_TO_RAD;
}

void initMPU6050() { //MPU6050 초기화 함수
    Wire.begin(I2C_SDA, I2C_SCL); //ESP32C3 지정 핀으로 I2C 통신 개시
    Wire.setClock(400000); // 400kHz Fast Mode 

    Wire.beginTransmission(MPU6050_ADDR); //전원 설정 전달 위한 통신 시작
    Wire.write(MPU6050_PWR_MGMT_1); Wire.write(0x00); //센서의 Sleep 모드 해제해서 깨움
    Wire.endTransmission(true); //I2c 전송 완료

    Wire.beginTransmission(MPU6050_ADDR); //DLPF 설정을 위한 통신 시작
    Wire.write(MPU6050_CONFIG); Wire.write(0x04); //0x04를 입력해 DLPF 대역폭을 20Hz 근처로 지정
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR); //자이로 범위 설정을 위한 통신 시작
    Wire.write(0x1B); Wire.write(0x08);  //0x08를 써서 자이로 측정 범위를 500deg/s로 설정
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR); //가속도 범위 설정을 위한 통신 시작
    Wire.write(MPU6050_ACCEL_CONFIG); Wire.write(0x08); //0x08을 써서 범위를 4G로 설정
    Wire.endTransmission(true);
}

void calibrateGyroOnly() { //정지 상태에서 자이로 영점 오프셋을 샘플링하는 함수
    float sumGx = 0, sumGy = 0, sumGz = 0; //자이로 축 누적 합을 담을 임시변수
    for (int i = 0; i < 500; i++) { //센서 값 500회 측정, 1초 동안 캘리브레이션
        float ax, ay, az, gx, gy, gz; //센서 읽기용 지역 변수
        readMPU6050(ax, ay, az, gx, gy, gz); //센서 raw 데이터 읽기
        sumGx += gx; sumGy += gy; sumGz += gz; //3축 자이로를 누적 가산
        delay(2); 
    }
    gx_offset = sumGx / 500.0f; //500회 누적값의 평균으로 3축 자이로 오프셋 결정
    gy_offset = sumGy / 500.0f; 
    gz_offset = sumGz / 500.0f;
}

// ==========================================================
// 4. setup() 및 loop()
// ==========================================================
unsigned long lastFilterUpdate = 0; //마지막 필터 연산 시각 저장을 위한 전역 변수 초기화

void setup() { //전원 공급시 실행되는 초기화 함수
    Serial.begin(115200); //시리얼 모니터 속도 115200bps
    initMPU6050(); 
    calibrateGyroOnly(); //자이로 500회 측정 후 캘리브레이션
    
    WiFi.mode(WIFI_STA); //EPS-NOW 통신을 위한 Wifi STA 모드
    esp_wifi_set_max_tx_power(56); //송신 신호 14dBm으로 세기 조절

    if (esp_now_init() != ESP_OK) {//EPS-NOW 프로토콜 초기화, 성공 여부 검사
        Serial.println("ESP-NOW Init Failed!");
        return;
    }

    memcpy(peerInfo.peer_addr, receiverAddress, 6); //사전 정의한 수신기 ESP32 MAC 주소 복사
    peerInfo.channel = 1; // 수신기 AP 기본 채널 1번에 고정
    peerInfo.encrypt = false; //고속 통신을 위한 암호화 해제
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer!");
    }

    lastFilterUpdate = micros(); //루프 진입 직전 현재시각으로 필터 연산 시점 기준값으로 사용
}

void loop() {
    unsigned long now = micros(); //us 단위 시스템 시각

    if (now - lastFilterUpdate >= 4000) { // 250Hz 루프 (4ms 주기)
        float dt = (now - lastFilterUpdate) / 1000000.0f; //마지막 주기로부터 경과한 시간을 s단위로 변환
        lastFilterUpdate = now; //필터 업데이트 시점 갱신

        float ax, ay, az, gx, gy, gz; //센서 측정값 임시 저장 지역 변수
        readMPU6050(ax, ay, az, gx, gy, gz); //MPU6050에서 가속도, 자이로 데이터 읽기
        
        filter.update(ax, ay, az, gx - gx_offset, gy - gy_offset, gz - gz_offset, dt); //Mahony 필터 연산

        payloadData.roll  = filter.roll_rad; //수신기 ESP32로 보낼 Roll/Pitch/3축 자이로/3축 가속도를 송신용 구조체에 담기
        payloadData.pitch = filter.pitch_rad;
        payloadData.gyroX = filter.gyroX_rads;
        payloadData.gyroY = filter.gyroY_rads;
        payloadData.gyroZ = filter.gyroZ_rads;
        payloadData.accX  = ax;
        payloadData.accY  = ay;
        payloadData.accZ  = az;

        esp_now_send(receiverAddress, (uint8_t *) &payloadData, sizeof(payloadData)); // 패킹된 구조체를 수신기로 ESP-NOW 전송
    }
}