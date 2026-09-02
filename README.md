# 2026ESWContest_WINCHER
하이브리드 윈치 제어기/WINCHER
---
WINCHER의 하이브리드 윈치 제어기는 ESP32 환경에서 슬라이딩모드 제어와 온디바이스 머신러닝으로 윈치의 진자운동을 댐핑하는 시스템입니다.

추가 모터 전력 소모나 임무 시간 증가 없이 진자운동 각도 평균제곱근오차 26% 감소, 최대각도 21% 감소,  진동에너지 48%를 감쇄시켰습니다.

---
파일 가이드
<br><br>
1. **esp32_winch_control**
 
   platformio.ini
   
   src/
   
   main.cpp
   
   wind_model_data.h
   <br><br> 

   
Vscode PlatformIO 환경에서 Espressif ESP32 Dev Module를 빌드 후 위 파일을 덮어씁니다.

메인 ESP32로 페이로드의 각도, 사용자의 임무 지령 등 모든 정보를 종합하여 모터를 제어합니다.

<br><br>
2. **mini_eps32c3_winch_payload**

   platformio.ini
   
   src/
   
   main.cpp
<br><br>
   

Vscode PlatformIO 환경에서  Seeed Studio XIAO ESP32C3를 빌드 후 위 파일을 덮어씁니다.

페이로드에 장착된 서브 EPS32C3으로 페이로드의 각도를 측정하여 메인 ESP32로 전송합니다.

<br><br>
3. **winch_gui_1.py**

VScode 환경에서 miniconda3로 파이썬 파일을 실행합니다. 코드 컴파일 후 요구하는 PyQt5 등을 설치해야 합니다. 

사용자 PC를 위한 그래픽사용자인터페이스 (GUI)으로 메인 ESP32로 임무지령을 보내고 실시간 텔레메트리를 수신합니다.

<br><br>
4. **tiny_ml_1.ipynb**

VScode 환경에서 Jupyter Notebook으로 실행합니다.

윈치 실시간 동작에 관여하지 않으며, ESP32 tinyML 모델 학습을 위한 파일입니다.

<br><br>
5. **data_analyzer_release.py**

VScode 환경에서 파이썬 파일을 실행합니다. 

윈치 실시간 동작에 관여하지 않으며, 윈치의 CSV 데이터를 분석하여 그래프를 만들고 계산하는 파일입니다.

<br><br>
6. **appendix2_tinyml_environment.pdf**
   
   보고서 부록2 입니다. 

<br><br> 
7. **appendix3_test_results.pdf**
   
   보고서 부록3입니다.
