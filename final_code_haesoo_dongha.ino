#include <SoftwareSerial.h>
SoftwareSerial mySerial(9, 8); // RX, TX
#define PIN(N) (1<<N)

float input_LR, input_FB;
float output_LR, output_FB;
bool direction_LR = true, direction_FB = true; // 일단 true
float lastInput_LR = 0, last_Input_FB = 0;
bool last_dir_LR, last_dir_FB;

unsigned long lastTime = 0;

float setpoint_LR = -3; 
float setpoint_FB = 7.40; //5.29
 
float error_LR = 0;
float last_error_LR = 0;

float error_FB = 0;
float last_error_FB = 0;

float P_out_LR, I_out_LR, D_out_LR;
float P_out_FB, I_out_FB, D_out_FB;

float Kp_LR = 30.0;
float Ki_LR = 10;
float Kd_LR = 1;

float Kp_FB = 5.835;// 11.5 앞으로 못잡음 14는 너무 센듯..
float Ki_FB = 80.0;//0.000045;//0.05;//0.1;//0.00025; // 10으로 하니까 좀 민감해진 느낌? 차이가 잘 안남
float Kd_FB = 0.0020;//0.077;//0.8;//0.000005;//0.6; 순간적인 넘어짐에 빠르게 대응하는듯 0.7-0.8 괜찮았다. 0.9 이상부터는 꽤 흔들림 (노이즈에 과하게 반응?)

//모터 세기 지정
float posMAX = 30, posMIN = 255; // 80~200 사이로 해봣는데 별로임 MIN 255로 늘리는게 확실히 좋음
float negMAX = -30, negMIN = -255;//70



// I2C 구현 //
#define READBIT 1
#define WRITEBIT 0

int16_t  rawacc_x,rawacc_y,rawacc_z;
int16_t  rawgyro_x,rawgyro_y,rawgyro_z;
int16_t  rawtemp;

float gyro_x_init, gyro_y_init, gyro_z_init;
int16_t data_array[7];

float Acc_x, Acc_y, Acc_z;
float Gyro_x, Gyro_y, Gyro_z;
float angle_x, angle_z, angle_y;

float last_x_angle, last_y_angle, last_z_angle;
float last_Gyro_x, last_Gyro_y, last_Gyro_z;

float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float accel_angle_x, accel_angle_y, accel_angle_z;

float caltime, pretime;

void TWI_Setup() {
  TWI_Init();
  TWI_Start(0x68, WRITEBIT);
  TWI_Write(0x6B);
  TWI_Write(0); 
  TWI_Stop();
}

void TWI_Init() {
  TWI0_MBAUD = 30;
  TWI0_MCTRLA |= TWI_ENABLE_bm;
  TWI0_MCTRLB |= TWI_FLUSH_bm; 
  TWI0_MSTATUS = (0x01<<0);
}


void TWI_Start(uint8_t addr, uint8_t d) { // d가 1이면 READ, 0이면 WRITE
  TWI0_MADDR = (addr << 1) + d;
  while (!(TWI0_MSTATUS & (TWI_WIF_bm | TWI_RIF_bm)));

}

void TWI_Write(uint8_t data) {
  while (!(TWI0_MSTATUS & TWI_WIF_bm));
  TWI0_MDATA = data;
  TWI0_MCTRLB = TWI_MCMD_RECVTRANS_gc;
  return (!(TWI0_MSTATUS & TWI_RXACK_bm));   
}

uint8_t TWI_Read(uint8_t isAck) {
  while (!(TWI0_MSTATUS & TWI_RIF_bm));
  uint8_t data = TWI0_MDATA;
  if (isAck == 0) {
    TWI0_MCTRLB = (TWI_ACKACT_ACK_gc  | TWI_MCMD_RECVTRANS_gc);
  }
  else {
    TWI0_MCTRLB = (TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc);
  }
  return data;
}

void TWI_Stop(void) {
  TWI0_MCTRLB = (TWI_MCMD_STOP_gc);
}


void Read_Accel(){
  
  TWI_Start(0x68, WRITEBIT); // write start
  TWI_Write(0x3B); // 가속도 레지스터
  TWI_Start(0x68, READBIT); // read start
  
  uint8_t data1, data2;
  for (int i = 0; i < 6; i++){
      data1 = TWI_Read(0);
      data2 = TWI_Read(0);
      data_array[i] = ((data1 << 8) | data2);
    }
    data1 = TWI_Read(0);
    data2 = TWI_Read(1); // nack
    data_array[6] = ((data1 << 8) | data2);
    
}

void filtering(){
  Read_Accel();
  
  rawacc_x = data_array[0];
  rawacc_y = data_array[1];
  rawacc_z = data_array[2];
  rawgyro_x = data_array[4];
  rawgyro_y = data_array[5];
  rawgyro_z = data_array[6];
  
   
  Acc_x = ((float)rawacc_x) / 16384;
  Acc_y = ((float)rawacc_y) / 16384;
  Acc_z = ((float)rawacc_z) / 16384;
  
  Gyro_x = ((float)rawgyro_x - gyro_x_init) / 131;
  Gyro_y = ((float)rawgyro_y - gyro_y_init) / 131;
  Gyro_z = ((float)rawgyro_z - gyro_z_init) / 131;
  
  accel_angle_y = atan(Acc_x / sqrt(pow(Acc_y, 2) + pow(Acc_z, 2))) * (-180) / 3.1415;
  accel_angle_x = atan(Acc_y / sqrt (pow(Acc_x, 2) + pow(Acc_z, 2))) * 180 / 3.1415;
  accel_angle_z = 0;
  
  caltime = (millis() - pretime) / 1000;
  
  gyro_angle_x += Gyro_x * caltime;
  gyro_angle_y += Gyro_y * caltime;
  gyro_angle_z += Gyro_z * caltime;
  float alpha = 0.98;
  
  angle_x = (alpha) * gyro_angle_x + (1 - alpha) * accel_angle_x;
  angle_y = (alpha) * gyro_angle_y + (1 - alpha) * accel_angle_y;
  angle_z = gyro_angle_z;

  last_x_angle = angle_x;
  last_y_angle = angle_y;
  last_z_angle = angle_z;

  pretime = millis();
  
}

void Cal_Initial() {
  float gyro_x_temp = 0, gyro_y_temp = 0, gyro_z_temp = 0;
  int16_t rawx, rawy, rawz;
  int16_t gyro_array[3];
  uint8_t data1, data2;
  for (int i = 0; i < 30; i++) {
    TWI_Start(0x68, 0); // 쓰기 시작
    TWI_Write(0x43); 
    TWI_Start(0x68, 1);
    for(int i = 0;i < 2;i++){
      data1 = TWI_Read(0); // ack
      data2 = TWI_Read(0);
      data_array[i] = ((data1 << 8) | data2);
    }
    data1 = TWI_Read(0); // ack
    data2 = TWI_Read(1); // nack

    data_array[2] = ((data1 << 8) | data2);
    gyro_x_temp += ((float) data_array[0]);
    gyro_y_temp += ((float) data_array[1]);
    gyro_z_temp += ((float) data_array[2]);
    
  }
  
  gyro_x_init = gyro_x_temp / 30.0;
  gyro_y_init = gyro_y_temp / 30.0;
  gyro_z_init = gyro_z_temp / 30.0;
}

void TCB_Init(){

PORTMUX_TCBROUTEA = (PORTMUX_TCB0_bm | PORTMUX_TCB1_bm);
TCA0_SINGLE_CTRLA = (TCA_SINGLE_CLKSEL_DIV64_gc| TCA_SINGLE_ENABLE_bm);

TCB0_CTRLA = (TCB_ENABLE_bm | TCB_CLKSEL_CLKTCA_gc);
TCB1_CTRLA = (TCB_ENABLE_bm | TCB_CLKSEL_CLKTCA_gc);

PORTF_DIR |= (PIN(4) | PIN(5));

TCB0_CTRLB = (TCB_CCMPEN_bm | TCB_CNTMODE_PWM8_gc);
TCB1_CTRLB = (TCB_CCMPEN_bm | TCB_CNTMODE_PWM8_gc);

}

void TCB_Reset(){
  
TCB0_CTRLA = 0;
TCB0_CTRLB = 0;
TCB0_CCMPH = 0;
TCB0_CCMPL = 0;
TCB1_CTRLA = 0;
TCB1_CTRLB = 0;
TCB1_CCMPH = 0;
TCB1_CCMPL = 0;

}

void PORT_Init(){
PORTE_DIR = PIN(0) | PIN(1) | PIN(2) | PIN(3);
PORTF_OUT |= (PIN(4) | PIN(5)); //PWM PIN
PORTD_DIR = PIN(6);
}

void FB_Motor(bool direction, int pwm, bool last_dir) { // 하단
  PORTE_OUTSET = 1 << 0;// 브레이크 해제
  PORTF_OUTSET = PIN(5);
  PORTF_OUT = PIN(5);
  if (direction) {
    if (last_dir != direction) {
      PORTE_OUTCLR = 1 << 0; // 브레이크
      PORTE_OUTSET = 1 << 0;// 브레이크 해제
    }
 //   PORTE_OUTSET = 1 << 1;// 브레이크 해제
      TCB1_CCMPH = 1;
      TCB1_CCMPL = 285 - pwm;
      PORTD_OUTCLR = 1 << 6; // 시계
      delay(1);
  }
else {
    if (last_dir != direction) {
        PORTE_OUTCLR = 1 << 0; // 브레이크
        PORTE_OUTSET = 1 << 0;// 브레이크 해제
    }
    TCB1_CCMPH = 1;
    TCB1_CCMPL = 285 - pwm;
    PORTD_OUTSET = 1 << 6; // 시계
   }
  }


void LR_Motor(bool direction, int pwm, bool last_dir) { // 리액션 휠
  //PORTE_OUTSET = 1 << 1;// 브레이크 해제
  PORTF_OUTSET = PIN(4);
  PORTF_OUT = PIN(4);
  if (direction) {
      PORTE_OUTCLR = 1 << 1; // 브레이크
      delayMicroseconds(200);
      PORTE_OUTSET = 1 << 1;// 브레이크 해제
      TCB0_CCMPH = 1;
      TCB0_CCMPL = 330 - pwm;
      PORTE_OUTCLR = 1 << 2; // 시계
      delayMicroseconds(2600);
  }
  else { 
      PORTE_OUTCLR = 1 << 1; // 브레이크
      delayMicroseconds(200);
      PORTE_OUTSET = 1 << 1;// 브레이크 해제
      TCB0_CCMPH = 1;
      TCB0_CCMPL = 330 - pwm;
      PORTE_OUTSET = 1 << 2; // 시계
      delayMicroseconds(2600);
  }
}


unsigned long lastTime_LR = 0;
unsigned long lastTime_FB = 0;

float PID_LR(float input){
    unsigned long now = millis();
    float dt = (now - lastTime_LR) / 1000.0; // 초 단위로 시간 간격 계산
    lastTime_LR = now;

    error_LR = setpoint_LR - input; // error = 기대값과 측정값의 오차
    //mySerial.print("현재 LR 오차: ");
    //mySerial.print(error_LR);
    P_out_LR = Kp_LR * error_LR;
    I_out_LR += (Ki_LR * error_LR) * dt; // 실제 dt 사용
    D_out_LR = (Kd_LR * (error_LR - last_error_LR)) / dt; // 실제 dt 사용

    last_error_LR = error_LR; // 마지막 오차 저장

    float output = P_out_LR + I_out_LR + D_out_LR;

    if (output > posMIN) output = posMIN;
    else if (output >= 0 && output < posMAX) output = posMAX;
    else if (output < negMIN) output = negMIN;
    else if (output < 0 && output > negMAX) output = negMAX;

    return output;
}

float PID_FB(float input){
    unsigned long now = millis();
    float dt = (now - lastTime_FB) / 1000.0; // 초 단위로 시간 간격 계산
    lastTime_FB = now;

    error_FB = setpoint_FB - input; // error = 기대값과 측정값의 오차
    P_out_FB = Kp_FB * error_FB;
    I_out_FB += (Ki_FB * error_FB) * dt; // 실제 dt 사용
    D_out_FB = (Kd_FB * (error_FB - last_error_FB)) / dt; // 실제 dt 사용

    last_error_FB = error_FB; // 마지막 오차 저장

    float output = P_out_FB + I_out_FB + D_out_FB;

    if (output > posMIN) output = posMIN;
    else if (output >= 0 && output < posMAX) output = posMAX;
    else if (output < negMIN) output = negMIN;
    else if (output < 0 && output > negMAX) output = negMAX;

    return output;
}

void setup(){
  
  mySerial.begin(19200);
  PORT_Init();
  TCB_Reset();
  TCB_Init();
  TWI_Init();
  TWI_Setup();
  delay(3000);
  Cal_Initial();

}
void loop(){
  filtering();
  
  input_LR = angle_x;
  input_FB = angle_y;
  
  output_LR = PID_LR(input_LR);
  output_FB = PID_FB(input_FB);

  if(output_LR > 0) {
    last_dir_LR = direction_LR;
    direction_LR = false;
  }
  else {
    last_dir_LR = direction_LR;
    direction_LR = true;
  }
  
  if(output_FB > 0) {
    last_dir_FB = direction_FB;
    direction_FB = false;
  }
  else {
    last_dir_FB = direction_FB;
    direction_FB = true;
  }
  LR_Motor(direction_LR, abs(output_LR), last_dir_LR); //계산된 output 만큼 모터 회전
  FB_Motor(direction_FB, abs(output_FB), last_dir_FB); //계산된 output 만큼 모터 회전

}
