//////////////////////////////////////////////////////////////
// Immersion Oil Heater Controller(Arduino Mega 2560)       //
// Modbus Master: ModbusMaster by Doc Walker                //
// Modbus Slave: ModbusRTUSlave by C.M.Buliner              //
// MAX31856:Adafruit_MAX31856 by Adafruit                   //
// Feedforward + PID Control, by br3ttb/PID@^1.2.1          //
//////////////////////////////////////////////////////////////

// Library
#include <ModbusMaster.h>               // ModbusMaster by Doc Walker, Ver.2.0.1
#include <ModbusRTUSlave.h>             // ModbusRTUSlave by C.M.Buliner, Ver.3.1.2
#include <Adafruit_MAX31856.h>          // Adafruit_MAX31856 by Adafruit Ver.1.2.8
#include <PID_v1.h>                     // PID Library by br3ttb/PID@^1.2.1

// 아두이노메가 시리얼통신 정의
#define DEBUG_SERIAL    Serial          // 시리얼 모니터 (디버깅)
#define HMI_SERIAL      Serial1         // Nextion HMI (디스플레이)
#define MASTER_SERIAL   Serial2         // RS485 Master (센서 및 장치)
#define SLAVE_SERIAL    Serial3         // RS485 Slave (PC/LabVIEW와 연결)
const unsigned long BAUD_RATE = 19200;  // 통신속도를 모두 통일
const byte SLAVE_UNIT_ID      = 1;      // 아두이노 자체의 슬레이브 ID를 1로 지정
const byte ID_SCR             = 1;      // 복수의 슬레이브를 제어하기 위해서, 각 슬레이브에 ID 부여, ID_SCR: 전력제어기(Pion) ID 지정

// Pion 전력제어기 관련 설정(모델: PION-L1W-025-01)
const uint16_t ADDR_RUN_CTRL = 0x00CA;      // Read/Write(Coils) Run/Stop 제어 주소 (DEC 202), 기본 RUN으로 설정됨(사용하지 않음).
const uint16_t ADDR_S_PHASE_CTRL = 0x0191;  // Read/Write(Holdings Registers) S상 출력 제어 주소 (DEC 401), 단상은 S상만 제어
const uint16_t ADDR_S_CURRENT_SA = 0x013D;  // Read Only(Input Registers), S 상 전류 (0.1 A Resolution)
int controlSCR_Value = 0;                   // 출력제어(0 ~ 100% to 0 ~ 10000, X 100), 1A 이상, Max 25A

// HMI 수신버퍼 및 인덱스 설정 (HMI에서 보내온 데이터 확인)
byte hmi_buffer[15];                       
int buf_idx = 0;

// Master 및 Slave 객체 생성
//Master 객체의 경우 슬레이브 마가 개별적으로 생성(슬레이브가 추가되는 경우, ModbusMaster masterXX를 추가(XX: 슬레이브 객체 이름))
ModbusMaster masterSCR;
ModbusRTUSlave slave(SLAVE_SERIAL);

// Slave Modbus 데이터 맵 (Holding Registers) 생성 및 파라미터 초기화
// index [0]:In_T(C), [1]:Out_T(C), [2]:Power(kW), [3]:SCR RUN/STOP (Write), [4]:Set_T(C) (Write)
uint16_t holdingRegs[10] = {0,};        // 실제 필요한 holdingRegs 갯수보다 여유있게 설정해야 함(에러방지)  
float In_T = 0;                         // In_T : 절연유체 입구 온도
float Out_T = 0;                        // Out_T : 절연유체출구 온도
float heaterPower = 0;                  // heaterPower : 히터 전력 제어값(0 to 10000)
float Set_T = 40;                       // Set_T : 절연유체 제어(목표) 온도
uint16_t StartStop = 0;                 // StartStop : 전력제어 출력 ON/OFF
bool sensorFault = false;               // sensorFault : 써모커플 고장(단선/접촉불량) 발생 시 true, 감지되면 제어 출력을 강제로 차단
//uint16_t lastStartStop = 0;           // lastSrtartStop : StartStop 파라미터가 변경되었을 때만 동작시키기 위해 초기값 설정 (PC 제어 고려)         
//uint16_t lastSet_T = 40;              // lastSet_T : Set_T 파라미터가 변경되었을 때만 동작시키기 위해 초기값 설정 (PC 제어 고려)

// MAX31856(써머커플 온도 측정 모듈) CS 핀 정의 및 객체 생성, 온도 측정 주기 설정
const int CS_IN_T = 48;                                     // In_T 측정용 MAX31856 CS 핀
const int CS_OUT_T = 49;                                    // Out_T 측정용 MAX31856 CS 핀
Adafruit_MAX31856 max_InT = Adafruit_MAX31856(CS_IN_T);     // CS_IN_T : 입구온도 측정 써머커플 CS Pin 번호
Adafruit_MAX31856 max_OutT = Adafruit_MAX31856(CS_OUT_T);   // CS_OUT_T : 출구온도 측정 써머커플 CS Pin 번호
unsigned long lastReadTime = 0;                             // lastReadTime : 일정 주기로 온도를 일기 위해 초기값 설정
const unsigned long sensorReadInterval = 1000;              // tempReadInterval: 온도 데이터 갱신 주기

// PID 제어 변수 및 파라미터 설정, 객체 생성
double Setpoint, Input, Output;
double Kp = 150.0, Ki = 0.15, Kd =  0.0;    // PID 파라미터 (P 150고, 온도차가 20이면 30 % 부하공급 (실험 결과에 의해 조정 필요), I는 최대한 작게하여 오버슈팅 최소화, D는 필요없음)
PID myController(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
double alpha = 200.0;                      // feedforward = (Set_T - In_T) * alpha, 제어온도 20도 차이면, alpha 값으로 40% 부하 공급 (실험 결과에 의해 조정 필요)

// 함수 선언
void sendToNextion(const char* componentId, float value);
void checkHMI();
void readSensors();
void parseNextionPacket(byte* buf, int len);

void setup() {
  DEBUG_SERIAL.begin(BAUD_RATE, SERIAL_8N1);    // 데이터 비트 8비트, 패리티 비트 None, 스탑비트 1비트, 8N1을 명시적으로 지정
  HMI_SERIAL.begin(BAUD_RATE, SERIAL_8N1);      // 데이터 비트 8비트, 패리티 비트 None, 스탑비트 1비트, 8N1을 명시적으로 지정
  MASTER_SERIAL.begin(BAUD_RATE, SERIAL_8N1);   // 데이터 비트 8비트, 패리티 비트 None, 스탑비트 1비트, 8N1을 명시적으로 지정
  SLAVE_SERIAL.begin(BAUD_RATE, SERIAL_8N1);    // 데이터 비트 8비트, 패리티 비트 None, 스탑비트 1비트, 8N1을 명시적으로 지정

  // Master 객체 초기화 (각 슬레이브에 고유 ID 지정됨), 슬레이브가 복수인 경우 슬레이브마다 객체 초기화
  masterSCR.begin(ID_SCR, MASTER_SERIAL);

  // Slave 설정 및 초기화
  // slave.configureHoldingRegisters(startAddress, numberOfRegisters)
  // slave.configureHoldingRegisters(holdingRegs, 10)의 경우는, 여기서는 holdingRegs 배열 자체를 Modbus Holding Register 영역으로 사용하고, 배열크기와 동일하게 지정
  slave.begin(SLAVE_UNIT_ID, BAUD_RATE);  
  slave.configureHoldingRegisters(holdingRegs, 10);  

  // Nextion HMI 초기 설정온도로 설정함(Nextion HMI 설정온도 객체는 Xfloat, x3)
  sendToNextion("x3", Set_T * 10);      // Nextion HMI는 소수점 1자리 표시이므로, 전송할 때만 10배로 변환     

  // MAX31856 센서 초기화 및 써모컬을 T타입으로 설정
  if (!max_InT.begin() || !max_OutT.begin()) {    // MAX31856 센서 초기화 실패 시(0), 성공하면 (1) 반환
    DEBUG_SERIAL.println(F("오류: MAX31856 센서를 찾을 수 없습니다. 결선을 확인하세요."));
  } else {
    max_InT.setThermocoupleType(MAX31856_TCTYPE_T);
    max_OutT.setThermocoupleType(MAX31856_TCTYPE_T);
  }

  // PID 초기화
  myController.SetOutputLimits(0, 10000);       // 출력제어(0 ~ 100% to 0 ~ 10000, X 100), 1A 이상, Max 25A
  myController.SetMode(AUTOMATIC);
  myController.SetSampleTime(1000);

  DEBUG_SERIAL.println("System Monitoring & Control Ready...");
}


void loop() {

  slave.poll();                                                // PC(Modbus Poll)로부터의 요청 처리 (메인 루프 맨처음에 마스터 요청 확인) 

  checkHMI();                                                  // Nextion HMI에서 보내온 데이터가 있는 지 확인 (Start/Stop 버튼 확인 및 수행)    

  if (millis() - lastReadTime >= sensorReadInterval) {        // 주기적인 온도 측정 및 전력변환기 출력 파워 측정 (주기설정: sensorReadInterval)
    lastReadTime = millis();
    readSensors();

    if (StartStop == 1 && !sensorFault) {
      // PID 제어 로직
      Input = Out_T;
      Setpoint = Set_T;
      myController.Compute();

      double feedforward = (Set_T - In_T) * alpha;                          // Feedforward 계산 (절연유체 입구온도 고려)
      controlSCR_Value = constrain((int)(Output + feedforward), 0, 10000);  // 최종 제어값

      // PID 제어값을 SCR(전력제어기)에 전송
      uint8_t result = masterSCR.writeSingleRegister(ADDR_S_PHASE_CTRL, controlSCR_Value);
      if (result == masterSCR.ku8MBSuccess) {
        // DEBUG_SERIAL.println("-> [성공] SCR ON");
      } else {
        DEBUG_SERIAL.print("-> [실패] 제어값 전송 실패. 에러 코드: 0x");
        DEBUG_SERIAL.println(result, HEX);
      }
    } else {
      if (StartStop == 1 && sensorFault) {
        DEBUG_SERIAL.println(F("안전정지: 써모커플 오류로 인해 히터 출력을 차단합니다."));
      }
      controlSCR_Value = 0;
      masterSCR.writeSingleRegister(ADDR_S_PHASE_CTRL, controlSCR_Value);
    }
  }
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Nextion 컴포넌트에 값을 전송하는 함수
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void sendToNextion(const char* componentId, float value) {
  int valToSend = (int)(value); 
  HMI_SERIAL.print(componentId);
  HMI_SERIAL.print(".val=");
  HMI_SERIAL.print(valToSend);
  HMI_SERIAL.write("\xFF\xFF\xFF", 3); // 종료 코드 3바이트 전송
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Nextion HMI 데이터를 읽고, 처리하는 함수
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void checkHMI() {
  static int ff_count = 0;

  while (HMI_SERIAL.available() > 0) {
    byte b = HMI_SERIAL.read();
    
    if (buf_idx < sizeof(hmi_buffer)) {      // 버퍼 오버플로우 방지 (배열 크기 안에서만 저장)
      hmi_buffer[buf_idx++] = b;             // 수신한 데이터를 현재 버퍼 인덱스에 저장하고, 인덱스를 1 증가시킴
    }
                             
    if (b == 0xFF) {                               // 종료 코드(0xFF) 카운트
      ff_count++;
      if (ff_count == 3) {                         // 종료 코드가 연속 3번 들어오면 패킷 분석 시작
        parseNextionPacket(hmi_buffer, buf_idx);   // 분석 완료 후 버퍼 및 카운터 초기화
        buf_idx = 0;
        ff_count = 0;
      }
    } else {
      ff_count = 0;                                // 연속되지 않으면 카운트 리셋
    }
  }
}  

void parseNextionPacket(byte* buf, int len) {         // 수신된 바이트 배열을 파싱하는 함수
  if (len < 4) return;                                // 최소 길이 검사 (종료코드 3바이트 제외하고 내용이 있어야 함)

  // START 버튼 처리 ("START"문자열 5바이트 + x3.val 4바이트 + 종료코드 3바이트 = 총 12바이트)
  if (len >= 12 && buf[0] == 'S' && buf[1] == 'T' && buf[2] == 'A' && buf[3] == 'R' && buf[4] == 'T') {    
    // "START" 바로 다음(인덱스 5, 6, 7, 8)에 오는 4바이트 숫자를 정수로 결합 (Little-Endian 방식)
    long x3_raw = 0;                                  // x3_raw: HMI x3.value 값 확인
    x3_raw |= ((long)buf[5]);                         // Nextion HMI에서는 print x3.val 명령은 4바이트(32비트) 정수 데이터를 전송
    x3_raw |= ((long)buf[6] << 8);                    // Little-Endian 방식으로 하위 바이트 부터 전송
    x3_raw |= ((long)buf[7] << 16);                   // 1바이트(8 bits)씩 순차적으로 저장 
    x3_raw |= ((long)buf[8] << 24);                   // ㅣ= (OR연산)

    Set_T = (float)x3_raw/10.0;                       // (int) : x3_raw 4바이트 크기 정수, 10으로 나누어 소수점 1자리로 변환하여 Set_T에 저장
    StartStop = 1;
    /* DEBUG_SERIAL.println("STATUS: RUNNING");
    DEBUG_SERIAL.print("Set_T: ");
    DEBUG_SERIAL.println(Set_T); */
  }
  
  // STOP 버튼 처리 ("STOP"문자열 4바이트 + 종료코드 3바이트 = 총 7바이트)
  else if (len >= 7 && buf[0] == 'S' && buf[1] == 'T' && buf[2] == 'O' && buf[3] == 'P') {
    StartStop = 0;
    /* DEBUG_SERIAL.println("STATUS: STOPPED"); */
  }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Pion 전력제어기에서 전류, MAX31856에서 온도를 읽어오는 함수
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void readSensors() {
  
  uint8_t result = masterSCR.readInputRegisters(ADDR_S_CURRENT_SA, 1);  // 전류값을 읽음 (Resolution: 0.1A), 10이 곱해져서 송신됨
  if (result == masterSCR.ku8MBSuccess) {
    heaterPower = (float)masterSCR.getResponseBuffer(0)/10 * 220;      // 10으로 나누어서 실제 전류 계산(A), 220V를 곱해서 출력(W) 계산
    sendToNextion("x2", heaterPower);                                  // Nextion에서 소수점 없이 W로 표시

    // 시리얼모니터로 전력제어기 출력 결과 출력
    /* DEBUG_SERIAL.print("Heater Power: ");
    DEBUG_SERIAL.print(heaterPower);
    DEBUG_SERIAL.println(" W");
  } else {
    // 통신 에러 발생 시 에러 코드 출력
    DEBUG_SERIAL.print("Modbus Error: 0x");
    DEBUG_SERIAL.println(result, HEX); */
  }

  // 각 센서로부터 섭씨 온도 읽기 (In_T, Out_T는 실제 온도(원래 스케일)로 저장하여 PID/Feedforward 계산과 Set_T가 같은 스케일을 갖도록 함)
  In_T = max_InT.readThermocoupleTemperature();
  sendToNextion("x0", In_T * 10);                // Nextion은 소수점 1자리 표시이므로, 전송할 때만 10배로 변환
  Out_T = max_OutT.readThermocoupleTemperature();
  sendToNextion("x1", Out_T * 10);               // Nextion은 소수점 1자리 표시이므로, 전송할 때만 10배로 변환

  // 에러 체크 및 시리얼 모니터 출력(디버그용으로, 필요시 주석 제거로 확인)
  /* DEBUG_SERIAL.print(F("[온도 모니터] In_T: "));
  if (isnan(In_T)) DEBUG_SERIAL.print(F("Error"));
  else DEBUG_SERIAL.print(In_T, 1);
  
  DEBUG_SERIAL.print(F(" °C | Out_T: "));
  if (isnan(Out_T)) DEBUG_SERIAL.println(F("Error"));
  else {
    DEBUG_SERIAL.print(Out_T, 1);
    DEBUG_SERIAL.println(F(" °C"));
  } */

  // 온도 데이터 오류가 발생하면, 시리얼모니터로 출력하고 제어 출력을 차단하기 위해 플래그 설정
  uint8_t fault_In = max_InT.readFault();                // MAX31856에서 Fault 발생 시, 0이 아닌 값 반환
  uint8_t fault_Out = max_OutT.readFault();              // MAX31856에서 Fault 발생 시, 0이 아닌 값 반환
  sensorFault = (fault_In != 0) || (fault_Out != 0);     // MAX31856에서 Fault 발생 시, 제어 출력을 차단하기 위해 sensorFault 플래그를 true로 설정
  if (sensorFault) {
    DEBUG_SERIAL.println(F("경고: 써모커플 접촉 불량 또는 단선 확인 필요"));
  }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++