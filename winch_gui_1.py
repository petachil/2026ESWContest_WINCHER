import sys
import socket
import struct
import time
import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGridLayout, QLabel, QPushButton, 
                             QDoubleSpinBox, QGroupBox, QCheckBox)
from PyQt5.QtCore import QThread, pyqtSignal
import pyqtgraph as pg

# ==========================================
# 1. Network UDP Receiver Thread
# ==========================================
class UDPReceiver(QThread): #Qt의 QThread 상속 받아 Worker Thread 클래스 정의
    data_received = pyqtSignal(dict) 

    def __init__(self, port=12345): #기본 수신 포트 12345 초기화
        super().__init__() #QThread 초기화 함수 호출
        self.port = port   #포트 번호를 클래스 내부 변수에 저장
        self.running = True 
    def run(self): #메인 루틴
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #IPv4, UDP 네트워크 소켓 생성
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        sock.bind(('0.0.0.0', self.port)) #소켓 바인딩
        sock.settimeout(0.1) #수신 대기 100ms 제한

        pkt_format = "<I f f f f f f f B f B" #Little endian 및 패킷 구조 규칙 정의
        pkt_size = struct.calcsize(pkt_format) #41byte

        while self.running:
            try:
                data, _ = sock.recvfrom(1024) #UDP 소켓에서 1024 Byte까지 읽어오기
                if len(data) == pkt_size:     #미리 계산한 패킷 크기와 일치 확인
                    unpacked = struct.unpack(pkt_format, data) #Tuple로 해석
                    payload = { #dict로 변환
                        "ts": unpacked[0],
                        "roll": unpacked[1],
                        "pitch": unpacked[2],
                        "gyroX": unpacked[3],
                        "gyroY": unpacked[4],
                        "theta_mag": unpacked[5],
                        "length": unpacked[6],
                        "L_dot": unpacked[7],
                        "smc_state": unpacked[8],
                        "tinyml_comp": unpacked[9],
                        "sd_status": unpacked[10]
                    }
                    self.data_received.emit(payload) #GUI 스레드로 Emit
            except socket.timeout: #소켓 타임아웃 예외 무시
                continue
            except Exception as e: #다른 예외는 에러 메시지
                print(f"UDP Recv Error: {e}")

        sock.close()

    def stop(self): #안전 종료
        self.running = False
        self.wait()

# ==========================================
# 2. Main GUI Window
# ==========================================
class WinchGUI(QMainWindow): #최상위 GUI 클래스 정의
    def __init__(self):
        super().__init__() #부모 클래스의 초기화 함수 호출
        self.setWindowTitle("SMC Winch Control Dashboard (STS3215 12V 45RPM Max)") #제목
        self.resize(1100, 880) #창 기본 크기 1100*880 픽셀

        self.esp32_ip = "192.168.4.1" #ESP32 AP의 기본 IP  주소
        self.cmd_port = 12346 #ESP32로 전송할 UDP 포트 번호
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #명령 송신용 IPv4/UDP 소켓 객체
        self.tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) #브로드캐스트 전송 패킷 출력 허용



        self.last_rx_time = time.time() #데이터 받은 시간 기록을 위한 현재 타임스탬프 저장
        self.time_history = [] #그래프에 그릴 시간, 진자 각도, 줄 길이의 데이터 리스트 초기화
        self.theta_history = []
        self.length_history = []
        self.max_pts = 200 #그래프 데이터 포인터 200개 제한

        self.init_ui() #화면 UI 위젯 배치 함수 호출

        self.rx_thread = UDPReceiver() #수신 전용 스레드 객체를 인스턴스화
        self.rx_thread.data_received.connect(self.update_telemetry) #수신 스레드의 data_received와 GUI의 update_telmemtry 를 연결
        self.rx_thread.start() #수신 스레드 백그라운드 동작

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        left_panel = QVBoxLayout() #좌측 레이아웃
        main_layout.addLayout(left_panel, 1)

        # [1] 긴급 정지 버튼
        self.btn_estop = QPushButton("🚨 긴급 정지 (MASTER E-STOP) ")
        self.btn_estop.setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 18px; height: 60px;")
        self.btn_estop.clicked.connect(self.send_estop)
        left_panel.addWidget(self.btn_estop)

        #  [2] 싨시간 텔레메트리 상태 
        status_box = QGroupBox("Real-Time Telemetry")
        status_layout = QGridLayout()
        status_box.setLayout(status_layout)

        self.lbl_net_status = QLabel("● Wi-Fi 연결 대기 중...")
        self.lbl_net_status.setStyleSheet("font-weight: bold; color: blue;")
        self.lbl_smc = QLabel("WAITING FOR ZEROING...")
        self.lbl_smc.setStyleSheet("font-weight: bold; color: orange;")
        self.lbl_roll = QLabel("0.00° / 0.00°")    #Roll/Pitch/Gryo/Theta/L/L_dot
        self.lbl_gyro = QLabel("0.00°/s / 0.00°/s")
        self.lbl_theta = QLabel("0.00°")
        self.lbl_length = QLabel("0.000 m")
        self.lbl_speed = QLabel("0.00 cm/s (0.000 m/s)")
        
        self.lbl_sd_hw_status = QLabel("○ 대기 / 미장착") #SD 카드
        self.lbl_sd_hw_status.setStyleSheet("font-weight: bold; color: gray;")

        status_layout.addWidget(QLabel("Network Link:"), 0, 0)
        status_layout.addWidget(self.lbl_net_status, 0, 1)
        status_layout.addWidget(QLabel("System State:"), 1, 0)
        status_layout.addWidget(self.lbl_smc, 1, 1)
        status_layout.addWidget(QLabel("Roll / Pitch:"), 2, 0)
        status_layout.addWidget(self.lbl_roll, 2, 1)
        status_layout.addWidget(QLabel("Gyro X / Y:"), 3, 0)
        status_layout.addWidget(self.lbl_gyro, 3, 1)
        status_layout.addWidget(QLabel("Theta Mag:"), 4, 0)
        status_layout.addWidget(self.lbl_theta, 4, 1)
        status_layout.addWidget(QLabel("Current Length (L):"), 5, 0)
        status_layout.addWidget(self.lbl_length, 5, 1)
        status_layout.addWidget(QLabel("Current Speed:"), 6, 0)
        status_layout.addWidget(self.lbl_speed, 6, 1)
        status_layout.addWidget(QLabel("SD Status:"), 7, 0)
        status_layout.addWidget(self.lbl_sd_hw_status, 7, 1)
        left_panel.addWidget(status_box)

        #  [3] SMC 제어기 변수 설정
        smc_box = QGroupBox("SMC Parameter Setup")
        smc_layout = QVBoxLayout()
        smc_box.setLayout(smc_layout)
        
        self.btn_smc_toggle = QPushButton("SMC 상태: ON (작동 중)") #토글 버튼
        self.btn_smc_toggle.setCheckable(True)
        self.btn_smc_toggle.setChecked(True)
        self.btn_smc_toggle.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; height: 30px;")
        self.btn_smc_toggle.clicked.connect(self.toggle_smc)
        smc_layout.addWidget(self.btn_smc_toggle)

        smc_param_layout = QHBoxLayout()
        self.spin_smc_angle = QDoubleSpinBox()
        self.spin_smc_angle.setRange(1.0, 20.0) #허용각 1도~20도, 초기값 4도, 증감 0.5도 단위
        self.spin_smc_angle.setValue(4.0)
        self.spin_smc_angle.setSingleStep(0.5)
        
        self.btn_update_smc = QPushButton("Update SMC Angle") #EPS32 전송 버튼
        self.btn_update_smc.setStyleSheet("background-color: #607D8B; color: white;")
        self.btn_update_smc.clicked.connect(self.send_smc_param)
        
        smc_param_layout.addWidget(QLabel("Tolerable Angle (deg):"))
        smc_param_layout.addWidget(self.spin_smc_angle)
        smc_param_layout.addWidget(self.btn_update_smc)
        smc_layout.addLayout(smc_param_layout)
        
        left_panel.addWidget(smc_box)

        # [4] SD Card 기록, 저장 설정
        sd_box = QGroupBox("SD Card Logging Control")
        sd_layout = QVBoxLayout()
        sd_box.setLayout(sd_layout)
        
        self.lbl_sd_filename = QLabel("저장 파일명: 기록 대기 중...")
        self.lbl_sd_filename.setStyleSheet("color: #607D8B; font-weight: bold; font-size: 13px;")
        sd_layout.addWidget(self.lbl_sd_filename)
        
        sd_btn_layout = QHBoxLayout()  #기록 버튼
        self.btn_sd_start = QPushButton("SD 카드 기록 시작")
        self.btn_sd_start.setStyleSheet("background-color: #008CBA; color: white; height: 35px;")
        self.btn_sd_start.clicked.connect(self.start_sd_log)
        
        self.btn_sd_stop = QPushButton("기록 완료 (파일 저장)") #저장 버튼
        self.btn_sd_stop.setStyleSheet("background-color: #f44336; color: white; height: 35px;")
        self.btn_sd_stop.clicked.connect(self.stop_sd_log)
        
        sd_btn_layout.addWidget(self.btn_sd_start)
        sd_btn_layout.addWidget(self.btn_sd_stop)
        sd_layout.addLayout(sd_btn_layout)
        left_panel.addWidget(sd_box)

        #  [5] 페이로드 IMU (Calibration) 및 자동 복귀
        cmd_box = QGroupBox("System Calibration")
        cmd_layout = QVBoxLayout()
        cmd_box.setLayout(cmd_layout)

        self.btn_zero = QPushButton("1. 수평 영점 캘리브레이션 (Unlock System)")
        self.btn_zero.setStyleSheet("background-color: #2196F3; color: white; height: 35px;")
        self.btn_zero.clicked.connect(self.send_zeroing)

        self.btn_home = QPushButton("2. 밀착 복귀 (Homing)")
        self.btn_home.setStyleSheet("background-color: #FF9800; color: white; height: 35px;")
        self.btn_home.clicked.connect(self.send_homing)

        cmd_layout.addWidget(self.btn_zero) #보정 버튼
        cmd_layout.addWidget(self.btn_home)
        left_panel.addWidget(cmd_box)

        # [6] 수동 조작 
        manual_box = QGroupBox("Manual Jog Control")
        manual_layout = QVBoxLayout()
        manual_box.setLayout(manual_layout)

        speed_input_layout = QHBoxLayout()
        self.spin_manual_speed = QDoubleSpinBox()
        self.spin_manual_speed.setRange(0.1, 2.22)  #속도 범위 0.1~2.22cm/s, 기본값 1.50cm/s
        self.spin_manual_speed.setValue(1.50)         
        self.spin_manual_speed.setSingleStep(0.1)    
        self.spin_manual_speed.setDecimals(2)

        speed_input_layout.addWidget(QLabel("Jog Speed (cm/s):"))
        speed_input_layout.addWidget(self.spin_manual_speed)
        manual_layout.addLayout(speed_input_layout)
        
        warning_layout = QVBoxLayout()
        self.lbl_warning = QLabel("⚠️ 소프트웨어 1.0m 스탑 제한 적용 중") #0m 강제 감기 경고 문구
        self.lbl_warning.setStyleSheet("color: #E91E63; font-weight: bold;")
        self.chk_override = QCheckBox("0m 제한 해제 (물리적 0m까지 강제 감기 허용)")
        self.chk_override.setStyleSheet("color: red; font-weight: bold;")
        warning_layout.addWidget(self.lbl_warning)
        warning_layout.addWidget(self.chk_override)
        manual_layout.addLayout(warning_layout)

        jog_btns_layout = QHBoxLayout()
        self.btn_up = QPushButton("올리기 (감기 -)") #상승, 하강, 정지 버튼
        self.btn_down = QPushButton("내리기 (풀기 +)")
        self.btn_stop = QPushButton("정지 (Stop)")
        
        self.btn_up.clicked.connect(lambda: self.send_manual(-abs(self.spin_manual_speed.value() * 0.01))) #올리기는 m/s 단위 음수로 변환
        self.btn_down.clicked.connect(lambda: self.send_manual(abs(self.spin_manual_speed.value() * 0.01)))#내리기는 m/s 단위 양수로 변환
        self.btn_stop.clicked.connect(lambda: self.send_manual(0.0)) #정지는 속도 0.0으로 지정

        jog_btns_layout.addWidget(self.btn_up)
        jog_btns_layout.addWidget(self.btn_down)
        jog_btns_layout.addWidget(self.btn_stop)
        manual_layout.addLayout(jog_btns_layout)
        
        self.btn_length_zero = QPushButton("길이 0m 업데이트 (길이 영점)") #줄의 길이를 0m로 잡는 영점 버튼
        self.btn_length_zero.setStyleSheet("background-color: #9C27B0; color: white; font-weight: bold; height: 30px;")
        self.btn_length_zero.clicked.connect(self.send_length_zero)
        manual_layout.addWidget(self.btn_length_zero)

        left_panel.addWidget(manual_box)

        # [7] 자동 미션 파라미터 설정
        auto_box = QGroupBox("Auto Mission Setup")
        auto_layout = QGridLayout()
        auto_box.setLayout(auto_layout)

        self.spin_target = QDoubleSpinBox()
        self.spin_target.setRange(0.05, 1.00) #목표 이동길이 범위 0.05m~1.00m, 기본값 0.50m
        self.spin_target.setValue(0.50)
        self.spin_target.setSingleStep(0.05)

        self.spin_speed = QDoubleSpinBox()  #목표 이동속도 범위 0.1cm/s~2.22cm/s, 기본값 1.50cm/s
        self.spin_speed.setRange(0.1, 2.22)
        self.spin_speed.setValue(1.50)
        self.spin_speed.setSingleStep(0.1)
        self.spin_speed.setDecimals(2)

        self.spin_hover = QDoubleSpinBox() #호버링 (목표 길이에서 정지하여 대기하는 시간) 범위 1.0~60.0s, 기본값 3.0s
        self.spin_hover.setRange(1.0, 60.0)
        self.spin_hover.setValue(3.0)

        self.btn_auto = QPushButton("Start Auto Mission") #미션 실행 버튼 바인딩
        self.btn_auto.setStyleSheet("background-color: #4CAF50; color: white; height: 35px;")
        self.btn_auto.clicked.connect(self.send_auto_mission)

        self.btn_mission_stop = QPushButton("🛑 Pause/Stop Mission") #미션 중단 버튼
        self.btn_mission_stop.setStyleSheet("background-color: #E91E63; color: white; height: 35px;")
        self.btn_mission_stop.clicked.connect(self.send_mission_stop)

        auto_layout.addWidget(QLabel("Target Length (m, max 1.0m):"), 0, 0)
        auto_layout.addWidget(self.spin_target, 0, 1)
        auto_layout.addWidget(QLabel("Speed (cm/s, max 2.22):"), 1, 0)
        auto_layout.addWidget(self.spin_speed, 1, 1)
        auto_layout.addWidget(QLabel("Hover Time (sec):"), 2, 0)
        auto_layout.addWidget(self.spin_hover, 2, 1)
        
        auto_layout.addWidget(self.btn_auto, 3, 0)
        auto_layout.addWidget(self.btn_mission_stop, 3, 1)
        left_panel.addWidget(auto_box)

        # 우측 실시간 그래프 시각화 영역
        right_panel = QVBoxLayout()
        main_layout.addLayout(right_panel, 2)

        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        self.plot_theta = pg.PlotWidget(title="Pendulum Angle (Theta Mag)") #진자의 합성 각도 그래프
        self.plot_theta.showGrid(x=True, y=True)
        self.curve_theta = self.plot_theta.plot(pen=pg.mkPen('r', width=2))

        self.plot_length = pg.PlotWidget(title="Winch Line Length (L)") #윈치의 줄 길이 그래프
        self.plot_length.showGrid(x=True, y=True)
        self.curve_length = self.plot_length.plot(pen=pg.mkPen('b', width=2))

        right_panel.addWidget(self.plot_theta)
        right_panel.addWidget(self.plot_length)

    # ==========================================
    # 3. 안전 송신 처리 함수
    # ==========================================
    def send_cmd_packet(self, cmd_type, p1=0.0, p2=0.0, p3=0.0): #ESP32 패킷 전송 공통 메서드
        try:
            pkt = struct.pack("<B f f f", cmd_type, float(p1), float(p2), float(p3)) #Little endian, data type
            self.tx_sock.sendto(pkt, (self.esp32_ip, self.cmd_port)) #ESP32 주소와 포트번호로 UDP 패킷 전송
        except OSError as e: #소켓 OS 에러 
            print(f"⚠️ [네트워크 송신 오류] ESP32에 명령 전달 실패: {e}")
            self.lbl_net_status.setText("⚠️ 송신 실패! (Wi-Fi 연결 확인 필요)")
            self.lbl_net_status.setStyleSheet("font-weight: bold; color: red;")

    def start_sd_log(self): #SD 카드 데이터 기록 명령 함수
        now = datetime.datetime.now()          #PC 시간/날짜 정보
        yymmdd = float(now.strftime("%y%m%d")) #날짜와 시간을 추출해 ESP32 전송용 float로 변환
        hhmm = float(now.strftime("%H%M"))   
        filename = f"winch_log_{now.strftime('%y%m%d%H%M')}.csv" #GUI에 CSV 파일 이름 표시 (ex winch_log_2608241920.csv)
        
        self.lbl_sd_filename.setText(f"기록 중: {filename}")  #r기록 중 표시
        self.lbl_sd_filename.setStyleSheet("color: blue; font-weight: bold; font-size: 13px;")
        
        self.send_cmd_packet(9, 1.0, yymmdd, hhmm) #ESP32에 SD 카드 로깅 시작 플래그, 날짜, 시간 정보 전달

    def stop_sd_log(self): #안전하게 중단 및 파일 닫기
        self.lbl_sd_filename.setText("기록 완료됨 (저장 안전 처리)")
        self.lbl_sd_filename.setStyleSheet("color: green; font-weight: bold; font-size: 13px;")
        self.send_cmd_packet(9, 0.0)  #ESP32에 SD 카드 로깅 중지 플래그 전달
 
    def send_zeroing(self): self.send_cmd_packet(1) #시스템 unlock, 영점 캘리브레이션 명령 전송
    def send_homing(self): self.send_cmd_packet(2)  #밀착 복귀 (Homing) 명령 전송 
    
    def send_manual(self, speed_ms): #수동 구동 명령을 보내는 함수
        override_flag = 1.0 if self.chk_override.isChecked() else 0.0 #강제 제한 해제 체크하면 1.0
        self.send_cmd_packet(3, speed_ms, override_flag)
        
    def send_length_zero(self): #줄 길이를 0m로 초기화 하는 함수
        self.send_cmd_packet(8) 
        self.chk_override.setChecked(False) #0m 제한 해제 체크박스를 다시 해제

    def send_estop(self): self.send_cmd_packet(5) #긴급 정지 (E-Stop) 전역 명령 전송, 모터 정지
    def send_mission_stop(self): self.send_cmd_packet(6) #진행 중인 자동 미션을 정지
    
    def toggle_smc(self): #SMC 제어 On/Off 토글 버튼 함수
        if self.btn_smc_toggle.isChecked():
            self.btn_smc_toggle.setText("SMC 상태: ON (작동 중)")
            self.btn_smc_toggle.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; height: 30px;")
        else:
            self.btn_smc_toggle.setText("SMC 상태: OFF (비활성화)")
            self.btn_smc_toggle.setStyleSheet("background-color: #9E9E9E; color: black; font-weight: bold; height: 30px;")
        self.send_smc_param()
        
    def send_smc_param(self): #설정한 SMC 허용각도를 Binary로 전송하는 함수
        if self.btn_smc_toggle.isChecked():
            angle = self.spin_smc_angle.value()
        else:
            angle = 90.0 #SMC가 Off 상태이면 허용각을 90도로 설정해 실질적으로 차단
        self.send_cmd_packet(7, angle) #허용 각도를 ESP32로 전송

    def send_auto_mission(self): #자동 미현 설정 함수
        t_len = self.spin_target.value() #목표 이동 길이 
        spd_ms = self.spin_speed.value() * 0.01 #입력된 cm/s를 m/s로 변환
        hover = self.spin_hover.value() #도달 후 대기할 호버링 시간
        self.send_cmd_packet(4, t_len, spd_ms, hover) #패킹하여 ESP32에 전달

    # ==========================================
    # 4. 수신 데이터 및 연결 체크 업데이트
    # ==========================================
    def update_telemetry(self, data): #UDPReceiver 수신 스레드에서 data를 받아서 화면을 업데이트하는 메서드
        self.last_rx_time = time.time() #타임아웃 확인용 타임스탬프를 현재 시각으로 갱신
        self.lbl_net_status.setText("● ESP32 Wi-Fi 연결됨")
        self.lbl_net_status.setStyleSheet("font-weight: bold; color: green;")

       
        if data['length'] <= 0.015 and self.btn_smc_toggle.isChecked(): # 줄 길이 1.5cm 이하면 GUI에서 SMC 자동 비활성화 (수납 시 과제어로 인한 파손 차단)
            self.btn_smc_toggle.setChecked(False)
            self.btn_smc_toggle.setText("SMC 상태: OFF (자동 중지: L <= 1.5cm)")
            self.btn_smc_toggle.setStyleSheet("background-color: #9E9E9E; color: black; font-weight: bold; height: 30px;")
            self.send_smc_param() #변경 각도를 ESP32로 업데이트

        self.lbl_roll.setText(f"{data['roll']*57.2958:.2f}° / {data['pitch']*57.2958:.2f}°") #Roll/Pitch를 rad에서 deg로 변환
        self.lbl_gyro.setText(f"{data['gyroX']*57.2958:.2f}°/s / {data['gyroY']*57.2958:.2f}°/s") #자이로 각속도를 rad에서 deg로 변환
        
        theta_deg = data['theta_mag'] * 57.2958 #진자 합성 기울기를 deg로 변환해 theta_deg에 할당
        self.lbl_theta.setText(f"{theta_deg:.2f}°")
        self.lbl_length.setText(f"{data['length']:.3f} m")
        
        spd_cms = data['L_dot'] * 100.0 #줄 이동 속도를 cm/s로 변환
        self.lbl_speed.setText(f"{spd_cms:.2f} cm/s ({data['L_dot']:.3f} m/s)")

        sd_stat = data.get('sd_status', 0) #Dictionary에서 sd_status 키 값 추출
        if sd_stat == 1:
            self.lbl_sd_hw_status.setText("● 정상 기록 중") #정상
            self.lbl_sd_hw_status.setStyleSheet("font-weight: bold; color: green;")
        elif sd_stat == 2:
            self.lbl_sd_hw_status.setText("✖ 카드 오류 / 미인식") #마운트 실패, 오류 경고문구
            self.lbl_sd_hw_status.setStyleSheet("font-weight: bold; color: red;")
            self.lbl_sd_filename.setText("⚠️ SD 카드 오류: 카드를 확인하세요.")
            self.lbl_sd_filename.setStyleSheet("color: red; font-weight: bold; font-size: 13px;")
        else:
            self.lbl_sd_hw_status.setText("○ 대기 / 미장착") #로깅 대기 
            self.lbl_sd_hw_status.setStyleSheet("font-weight: bold; color: gray;")

        if data['smc_state'] == 3:
            self.lbl_smc.setText("🚨 E-STOP ACTIVE (Zeroing Required)") #비상정지, 재영점 안내
            self.lbl_smc.setStyleSheet("font-weight: bold; color: white; background-color: red; padding: 5px;")
        elif data['smc_state'] == 2:
            self.lbl_smc.setText("WAITING FOR ZEROING... (Locked)") #영점 대기 중 
            self.lbl_smc.setStyleSheet("font-weight: bold; color: orange;")
        elif data['smc_state'] == 1:
            self.lbl_smc.setText("ACTIVE (Supervisory Damping)") #SMC 가동 중
            self.lbl_smc.setStyleSheet("font-weight: bold; color: red;")
        else:
            self.lbl_smc.setText("SAFE (Normal)") #SMC 해제 후 단순 구동 중
            self.lbl_smc.setStyleSheet("font-weight: bold; color: green;")

        now = time.time() #그래프 x축 시간 계산을 위한 타임스탬프 취득
        self.time_history.append(now)
        self.theta_history.append(theta_deg)
        self.length_history.append(data['length']) #타임 스탬프, 진자 각도, 줄 길이 데이터를 리스트 배열 끝에 추가

        if len(self.time_history) > self.max_pts: #바퍼에 누적된 데이터 수가 200개 초과 시 오래된 데이터 제거 (pop(0))
            self.time_history.pop(0)
            self.theta_history.pop(0)
            self.length_history.pop(0)

        t_base = [t - self.time_history[0] for t in self.time_history] #그래프 x축이 절대 시간이 아닌 가장 오래된 데이터 기준으로 시작 되도록 t_base 생성
        self.curve_theta.setData(t_base, self.theta_history) #Pyqtgraph의 실시간 곡선 객체에 새로운 데이터를 할당해 그래프를 다시 그림
        self.curve_length.setData(t_base, self.length_history)

    def closeEvent(self, event): #GUI 창 닫을 시 Event handler
        self.rx_thread.stop() #수신스레드 (UDPReceiver)내의 수신 루프 안전 정지
        event.accept() #앱 정상 종료

if __name__ == "__main__": #직접 실행할때만 실행
    app = QApplication(sys.argv) #QApplication 인스턴스 생성
    window = WinchGUI() #윈도우 클래스 화면객체 생성
    window.show() #생성된 창을 모니터에 표시
    sys.exit(app.exec_()) #이벤트 루프 구동, 창 닫으면 안전하게 종료