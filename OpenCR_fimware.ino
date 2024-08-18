#include <Dynamixel2Arduino.h>
#include <IMU.h>

#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial1  //serial 포트
//#define DEBUG_SERIAL Serial  //USB

#define LS_DXL_ID 102
#define LE_DXL_ID 104
#define LW_DXL_ID 106
#define RS_DXL_ID 103
#define RE_DXL_ID 105
#define RW_DXL_ID 107

#define RWH1_DXL_ID 6
#define RWH2_DXL_ID 4
#define LWH1_DXL_ID 1
#define LWH2_DXL_ID 3

#define DXL_DIR_PIN 84  // OpenCR Board's DIR PIN.
#define DXL_PROTOCOL_VERSION 1.0

#define TRIG_LEFT 2       
#define ECHO_LEFT 3 
#define TRIG_RIGHT 4 
#define ECHO_RIGHT 5


#define MIDDLE_ID 42
#define MOUNT_ID 41
#define HEAD_ID 40

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

int DXL_ID_ARM[] = { LS_DXL_ID, LE_DXL_ID, LW_DXL_ID, RS_DXL_ID, RE_DXL_ID, RW_DXL_ID };
int DXL_ID_WH[] = { RWH1_DXL_ID, RWH2_DXL_ID, LWH2_DXL_ID, LWH1_DXL_ID };
int rot_vel;
unsigned int Forward_BEAT = 0,Back_BEAT = 0,Stop_BEAT = 1, Bending_forward_BEAT=0, Bending_back_BEAT=0, Position_Flag=0, Serial_Flag=0;
char seread=NULL;
long distance_left_sum=0;
long distance_right_sum=0;
int cm_LEFT, cm_RIGHT;


int hang_degree[]={150,275,155,150,275,155};             //기본값
int binding_degree[]={150,275,225,150,275,225};          //집기 각도
int turn_degree1[]={150,275,225,210,85,270};             //뒤집기 오른팔
int turn_degree2[]={210,85,270,210,85,270};               //뒤집기 왼팔

int bias_degree_1[]={150,240,235,210,85,285};              //오른팔 안쪽으로 돌리기
int bias_degree_2[]={150,240,155,210,85,190};               // 왼팔, 오른팔 풀기
int bending_degree_1[]={230,240,155,210,85,190};               //1초 먼저 왼어깨 오른어깨 꺽은 후
int bending_degree_2[]={220,255,155,190,85,200};                 //나머지 꺽기
int bending_degree_3[]={170,255,155,155,85,200};                
int bending_degree_4[]={170,255,210,100,85,200};

int bending_degree_5[]={150,260,210,180,120,210};           //forward 꺽기4 각도 목표값
int bias_degree_3[]={150,280,210,220,70,140};              //왼팔 오른팔 집기
int bias_degree_4[]={150,275,225,150,275,100};              //원래대로


//짐벌 제어 변수
int DXL_all[] = {MIDDLE_ID, MOUNT_ID, HEAD_ID};
int patrol_angle[] = {92, 182, 272, 182};
char gimbal_dir = 0;                 

int q_flag = 0 ; //젯슨나노 송신용 플래그비트

cIMU    IMU;

void cam_init_setting() { //짐벌 초기 세팅
  dxl.setGoalPosition(MOUNT_ID, 92, UNIT_DEGREE);           //하부 마운트 
  dxl.setGoalPosition(MIDDLE_ID, 90, UNIT_DEGREE);          //중간 프레임 
  dxl.setGoalPosition(HEAD_ID, 182, UNIT_DEGREE);            //헤드 프레임 
}

void cam_patrol_mode() { //순찰모드
  dxl.setGoalPosition(MOUNT_ID, patrol_angle[gimbal_dir], UNIT_DEGREE);            //하부 마운트 ,183도,도단위사용
  dxl.setGoalPosition(MIDDLE_ID, 90, UNIT_DEGREE);
  dxl.setGoalPosition(HEAD_ID, 182, UNIT_DEGREE);
  gimbal_dir++;
  if(gimbal_dir>3) gimbal_dir=0;
}

void position_setting(int degree[6], int delay_time = 0) {
  for (int i = 0; i < 6; i++)  dxl.setGoalPosition(DXL_ID_ARM[i], degree[i], UNIT_DEGREE);
  if (delay_time > 10) {
    delay(delay_time);
  }
}

int* in_back(int degree[6]) {         
  static int in_back_degree[6]={0};

  in_back_degree[0]=360-degree[3];
  in_back_degree[1]=360-degree[4];
  in_back_degree[2]=360-degree[5];
  in_back_degree[3]=360-degree[0];
  in_back_degree[4]=360-degree[1];
  in_back_degree[5]=360-degree[2];
  return in_back_degree;
}

int dis_LEFT() {
    long duration_LEFT, distance_LEFT;
    digitalWrite(TRIG_LEFT, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_LEFT, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_LEFT, LOW);
    duration_LEFT = pulseIn(ECHO_LEFT, HIGH);
    distance_LEFT = duration_LEFT * 17 / 1000;
    return distance_LEFT;
}
int dis_RIGHT() {
    long duration_RIGHT, distance_RIGHT;
    digitalWrite(TRIG_RIGHT, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_RIGHT, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_RIGHT, LOW);
    duration_RIGHT = pulseIn(ECHO_RIGHT, HIGH);
    distance_RIGHT = duration_RIGHT * 17 / 1000;
    return distance_RIGHT;
}


void Forward(int goal_rot_vel_value=0) {             //앞으로
  if (Forward_BEAT == 1) {           //기본값 1
    for (rot_vel= 0; rot_vel <= goal_rot_vel_value; ++rot_vel) {  
      dxl.setGoalVelocity(RWH1_DXL_ID, -rot_vel);
      dxl.setGoalVelocity(RWH2_DXL_ID, rot_vel);
      dxl.setGoalVelocity(LWH1_DXL_ID, -rot_vel);
      dxl.setGoalVelocity(LWH2_DXL_ID, rot_vel);
      delay(2);
    }
    Forward_BEAT = 2;
    Back_BEAT = 0; 
    Bending_forward_BEAT=1; 
    Bending_back_BEAT=0;       //이게 맞나??
    Stop_BEAT = 1;
    
  } 
  else if (Forward_BEAT == 2) {
    dxl.setGoalVelocity(RWH1_DXL_ID, -rot_vel);
    dxl.setGoalVelocity(RWH2_DXL_ID, rot_vel);
    dxl.setGoalVelocity(LWH1_DXL_ID, -rot_vel);
    dxl.setGoalVelocity(LWH2_DXL_ID, rot_vel);
  } 
  else;
}

void Back(int goal_rot_vel_value=0) {                  //뒤로
  if (Back_BEAT == 1) {
    for (rot_vel= 0; rot_vel <= goal_rot_vel_value; ++rot_vel) {
      dxl.setGoalVelocity(RWH1_DXL_ID, rot_vel);
      dxl.setGoalVelocity(RWH2_DXL_ID, -rot_vel);
      dxl.setGoalVelocity(LWH1_DXL_ID, rot_vel);
      dxl.setGoalVelocity(LWH2_DXL_ID, -rot_vel);
      delay(2);
    }
    Forward_BEAT = 0;
    Back_BEAT = 2;
    Bending_forward_BEAT=0;           //이게 맞나??
    Bending_back_BEAT = 1;
    Stop_BEAT = 1;
  }
  else if (Back_BEAT == 2) {
    dxl.setGoalVelocity(RWH1_DXL_ID, rot_vel);
    dxl.setGoalVelocity(RWH2_DXL_ID, -rot_vel);
    dxl.setGoalVelocity(LWH1_DXL_ID, rot_vel);
    dxl.setGoalVelocity(LWH2_DXL_ID, -rot_vel);
  } else;
}

void STOP() {  //정지
  if (Stop_BEAT == 1) {  
    for (; rot_vel>=0; --rot_vel) { 
      if(Back_BEAT==2){           //back 로 가는 중
        dxl.setGoalVelocity(RWH1_DXL_ID, rot_vel);
        dxl.setGoalVelocity(RWH2_DXL_ID, -rot_vel);
        dxl.setGoalVelocity(LWH1_DXL_ID, rot_vel);
        dxl.setGoalVelocity(LWH2_DXL_ID, -rot_vel);
      }
      if(Forward_BEAT==2){           //forward 로 가는 중
        dxl.setGoalVelocity(RWH1_DXL_ID, -rot_vel);
        dxl.setGoalVelocity(RWH2_DXL_ID, rot_vel);
        dxl.setGoalVelocity(LWH1_DXL_ID, -rot_vel);
        dxl.setGoalVelocity(LWH2_DXL_ID, rot_vel);
      }
      delay(2);
    }
    Stop_BEAT = 0;
    Forward_BEAT = 1;
    Back_BEAT = 1;
  } 
  else if (Stop_BEAT == 0) {
    dxl.setGoalVelocity(RWH1_DXL_ID, 0);
    dxl.setGoalVelocity(RWH2_DXL_ID, 0);
    dxl.setGoalVelocity(LWH1_DXL_ID, 0);
    dxl.setGoalVelocity(LWH2_DXL_ID, 0);
  }
}


void Bending_forward_1(){
  dxl.writeControlTableItem(P_GAIN, LW_DXL_ID, 8);          
  dxl.writeControlTableItem(P_GAIN, RW_DXL_ID, 8);     

  if(Bending_forward_BEAT == 1 && dxl.getPresentPosition(LW_DXL_ID,UNIT_DEGREE)>156) {
    STOP();     
    Back(80);
    delay(2000); 
    STOP();

    dxl.writeControlTableItem(P_GAIN, LW_DXL_ID, 32);                 //초기값
    dxl.writeControlTableItem(P_GAIN, RW_DXL_ID, 32);                 //초기값

    position_setting(binding_degree,2000);                          //집기
    position_setting(bias_degree_1,3500);                           //오른팔 안쪽으로 돌리기
    position_setting(bias_degree_2,2000);                           //왼팔, 오른팔 풀기  

    dxl.writeControlTableItem(MOVING_SPEED, LS_DXL_ID, 65);   
    dxl.writeControlTableItem(MOVING_SPEED, RS_DXL_ID, 65);  

    Forward(80); 
    delay(2000); 
    position_setting(bending_degree_1,2800);           //꺽기1 
    position_setting(bending_degree_2,2200);           //꺽기2
    position_setting(bending_degree_3,2000);           //꺽기3
    position_setting(bending_degree_4,1000);                //꺽기4


    Bending_forward_BEAT=2;
  }
}

void Bending_forward_2(){
  dxl.writeControlTableItem(P_GAIN, LW_DXL_ID, 8);          
  dxl.writeControlTableItem(P_GAIN, RW_DXL_ID, 8);       

  if(Bending_forward_BEAT == 2 && dxl.getPresentPosition(RW_DXL_ID,UNIT_DEGREE)>201) {      
    dxl.writeControlTableItem(P_GAIN, LW_DXL_ID, 32);                  //원래대로 
    dxl.writeControlTableItem(P_GAIN, RW_DXL_ID, 32);                  //원래대로
    position_setting(bending_degree_5,4500);    

    dxl.writeControlTableItem(MOVING_SPEED, LS_DXL_ID, 150);                 //원래대로
    dxl.writeControlTableItem(MOVING_SPEED, RS_DXL_ID, 150);                 //원래대로  
    STOP();
    position_setting(bias_degree_3,2000);                           //오른팔 집기
    position_setting(bias_degree_4,3500);                           //원래대로, 바퀴 밖으로

    position_setting(hang_degree,2000);                           //처음 모습으로

    Bending_forward_BEAT =0;
  }
}

void Bending_back_1(){
  dxl.writeControlTableItem(P_GAIN, LW_DXL_ID, 8);          
  dxl.writeControlTableItem(P_GAIN, RW_DXL_ID, 8);     

  if(Bending_back_BEAT == 1 && dxl.getPresentPosition(RW_DXL_ID,UNIT_DEGREE)<204) {
    STOP();     
    Forward(80);
    delay(2000); 
    STOP();

    dxl.writeControlTableItem(P_GAIN, LW_DXL_ID, 32);                 //초기값
    dxl.writeControlTableItem(P_GAIN, RW_DXL_ID, 32);                 //초기값

    position_setting(in_back(binding_degree),2000);                          //집기
    position_setting(in_back(bias_degree_1),3500);                           //오른팔 안쪽으로 돌리기
    position_setting(in_back(bias_degree_2),2000);                           //왼팔, 오른팔 풀기  

    dxl.writeControlTableItem(MOVING_SPEED, LS_DXL_ID, 65);   
    dxl.writeControlTableItem(MOVING_SPEED, RS_DXL_ID, 65);  

    Back(80); 
    delay(2000); 
    position_setting(in_back(bending_degree_1),2800);           //꺽기1 
    position_setting(in_back(bending_degree_2),2200);           //꺽기2
    position_setting(in_back(bending_degree_3),2000);           //꺽기3
    position_setting(in_back(bending_degree_4),1000);           //꺽기4

    Bending_back_BEAT=2;
  }
}

void Bending_back_2(){
  dxl.writeControlTableItem(P_GAIN, LW_DXL_ID, 8);          
  dxl.writeControlTableItem(P_GAIN, RW_DXL_ID, 8);       

  if(Bending_back_BEAT == 2 && dxl.getPresentPosition(LW_DXL_ID,UNIT_DEGREE)<159) {      
    dxl.writeControlTableItem(P_GAIN, LW_DXL_ID, 32);                  //원래대로 
    dxl.writeControlTableItem(P_GAIN, RW_DXL_ID, 32);                  //원래대로
    position_setting(in_back(bending_degree_5),4500);    

    dxl.writeControlTableItem(MOVING_SPEED, LS_DXL_ID, 150);                 //원래대로
    dxl.writeControlTableItem(MOVING_SPEED, RS_DXL_ID, 150);                 //원래대로  
    STOP();
    position_setting(in_back(bias_degree_3),2000);                           //오른팔 집기
    position_setting(in_back(bias_degree_4),3500);                           //원래대로, 바퀴 밖으로

    position_setting(in_back(hang_degree),2000);                           //처음 모습으로

    Bending_back_BEAT =0;
  }
}

void setup() {
  DEBUG_SERIAL.begin(115200);         // 통신 속도 설정
  //DEBUG_SERIAL2.begin(115200);

  IMU.begin();

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  //다이나믹셀 설정
  for (int i = 0; i < 6; i++) {
    dxl.torqueOff(DXL_ID_ARM[i]);
    dxl.setOperatingMode(DXL_ID_ARM[i], OP_POSITION);  
    dxl.torqueOn(DXL_ID_ARM[i]);
    dxl.writeControlTableItem(CW_ANGLE_LIMIT, DXL_ID_ARM[i], 0);     //MX64 초기설정
    dxl.writeControlTableItem(CCW_ANGLE_LIMIT, DXL_ID_ARM[i], 4095);     //MX64 초기설정
    dxl.writeControlTableItem(MOVING_SPEED, DXL_ID_ARM[i], 150);     //position 속도 설정, 약 200*0.111=22.2[rev/min]         원래는 최대 출력 59[rev/min]이였음.
  }
  for (int i = 0; i < 4; i++) {
    dxl.torqueOff(DXL_ID_WH[i]);
    dxl.setOperatingMode(DXL_ID_WH[i], OP_VELOCITY);  
    dxl.torqueOn(DXL_ID_WH[i]);
  }
  for (int i = 0; i < 3; i++) {
    dxl.torqueOff(DXL_all[i]);
    dxl.setOperatingMode(DXL_all[i], OP_POSITION);
    dxl.torqueOn(DXL_all[i]);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_all[i], 80);
  }  

  dxl.writeControlTableItem(CCW_COMPLIANCE_SLOPE, RW_DXL_ID, 128);       //집을때 유연선 증가
  dxl.writeControlTableItem(CCW_COMPLIANCE_SLOPE, LW_DXL_ID, 128);       //집을때 유연선 증가
  dxl.writeControlTableItem(CW_COMPLIANCE_SLOPE, RW_DXL_ID, 128);       //집을때 유연선 증가
  dxl.writeControlTableItem(CW_COMPLIANCE_SLOPE, LW_DXL_ID, 128);       //집을때 유연선 증가


  position_setting(hang_degree);
  cam_init_setting(); //짐벌 자세 초기 세팅
  Position_Flag=1; 
  STOP();
}

void loop() {
  //통신부
  if (q_flag==0){
    DEBUG_SERIAL.println('q');   //q를 젯슨나노에 송신
    q_flag = 1;                //동작을 끝내기 전까지 질문금지(연속 송신 방지)
  }
  else;                     


  //짐벌부
  if ((DEBUG_SERIAL.available() > 0) && (q_flag == 1)) { //q_flag==1일 때 수신모드
    seread = DEBUG_SERIAL.read();
    if (seread == 'n'){
      cam_patrol_mode();
      Serial_Flag = 3;
     } 
    else if (seread == 's'&&((Bending_back_BEAT==1)||(Bending_forward_BEAT==1))){
      Serial_Flag = 2;
    }
    else if (seread == 'b'){
      dxl.setGoalPosition(MOUNT_ID, 2, UNIT_DEGREE);
      Serial_Flag = 2;
    }
    else if (seread == 'l'){
      dxl.setGoalPosition(MOUNT_ID, 92, UNIT_DEGREE);
      Serial_Flag = 2;
    }
    else if (seread == 'f'){
      dxl.setGoalPosition(MOUNT_ID, 182, UNIT_DEGREE);
      Serial_Flag = 2;
    }
    else if (seread == 'r'){
      dxl.setGoalPosition(MOUNT_ID, 272, UNIT_DEGREE);
      Serial_Flag = 2;
    }
    else;
    q_flag = 0;
  }
  else;


  //관절부
  if (Serial_Flag == 2){                         //stop
    STOP();    
  }
  else if (Serial_Flag == 3) {                          //normal
      distance_left_sum=0;
      distance_right_sum=0;
      //cm_LEFT = dis_LEFT();
      //cm_RIGHT = dis_RIGHT();
      for(int i=0;i<10;i++)
      {
        cm_LEFT = dis_LEFT();
        cm_RIGHT = dis_RIGHT();
        distance_left_sum+=cm_LEFT;
        distance_right_sum+=cm_RIGHT;
      }
      distance_left_sum=distance_left_sum/10;
      distance_right_sum=distance_right_sum/10;

      //DEBUG_SERIAL.println(cm_LEFT);
      //DEBUG_SERIAL.println(cm_RIGHT);
      

      if ((distance_left_sum <= 15) && (Forward_BEAT == 2)&&(Bending_forward_BEAT==1)){    //앞으로 가다 뒤로 turn
        dxl.writeControlTableItem(P_GAIN, LW_DXL_ID, 32);                  //원래대로 
        dxl.writeControlTableItem(P_GAIN, RW_DXL_ID, 32);                  //원래대로  
        
        STOP();
        position_setting(binding_degree,2000);
        position_setting(turn_degree1,3500);
        position_setting(turn_degree2,3500);
        position_setting(in_back(hang_degree),2000);
        Position_Flag = 2;
        Back(80);
      }
      else if ((distance_right_sum <= 15) && (Back_BEAT == 2)&&(Bending_back_BEAT==1)){     //뒤로 가다 앞으로 turn
        dxl.writeControlTableItem(P_GAIN, LW_DXL_ID, 32);                  //원래대로 
        dxl.writeControlTableItem(P_GAIN, RW_DXL_ID, 32);                  //원래대로   

        STOP();
        position_setting(in_back(binding_degree),2000);
        position_setting(in_back(turn_degree1),3500);
        position_setting(in_back(turn_degree2),3500);
        position_setting(hang_degree,3500);       
        Position_Flag = 1;
        Forward(80);
      }
      else{
        if (Position_Flag == 1){
          Forward(80);
          Bending_forward_1();
          Bending_forward_2();
        } 
        else if (Position_Flag == 2){
          Back(80);
          Bending_back_1();
          Bending_back_2();
        } 
      }
    }
  else;
}
