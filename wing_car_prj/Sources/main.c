/*
 *######################################################################
 *                                RAppIDJDP
 *           Rapid Application Initialization and Documentation Tool
 *                         Freescale Semiconductor Inc.
 *
 *######################################################################
 *
 * Project Name           : TRK_MPC5606B_Example
 *
 * Project File           : TRK_MPC5606B_Example.rsp
 *
 * Revision Number        : 1.0
 *
 * Tool Version           : 1.2.1.5
 *
 * file                   : main.c
 *
 * Target Compiler        : Codewarrior
 *
 * Target Part            : MPC5606B
 *
 * Part Errata Fixes      : none
 *
 * Project Last Save Date : 27-Jun-2015 22:27:24
 *
 * Created on Date        : 27-Jun-2015 22:27:28
 *
 * Brief Description      : This file contains main() function call.
 *
 ******************************************************************************** 
 *
 * Detail Description     : This file contains main() routine which calls system
 *                         initialization routine and interrupt enable routine if defined.
 *
 ******************************************************************************** 
 *
 *######################################################################
 */

/*********************** pin's data******************************/////////
/* 
emios ch 0   : SERVO ANGLE INPUT
emios ch 2,3 : MOTOR RIGHT  ch0,1
emios ch 4,5 : MOTOR LEFT ch0,1

GPIO ch 15    : MOTOR(DC) ENABLE
GPIO ch 28,29 : CAMERA1(LEFT) SI ,CLK
GPIO ch 108,109: CAMERA2(RIGHT) SI ,CLK
GPIO CH    64  : CAR START/END SW

ADC_ADP_0     : CAMERA1(LEFT) OUTPUT
ADC_ADP_1     : CAMERA1(RIGHT) OUTPUT


Global Prescaler Divide Ratio 25 ->2.56MHz Prescaler Frequency
 */


/********************  Dependent Include files here **********************/

#include "rappid_ref.h"
#include "rappid_utils.h"
#include "sys_init.h"
#include "CANapi.h"
#include "sbc_hld.h"
#include "gpio_drv.h"
#include "pot_hld.h"
#include "photo_sensor_hld.h"
#include "adc_drv.h"
#include "emios_init.h"
#include "intc_pit.h"
#include "gpio_drv.h"

/**********************  Function Prototype here *************************/

void main(void);
void getCamera(void);         // 카메라 값을 받아오기 위한 함수
void leftCameraFilter(void);   // 왼쪽카메라에서 받아온 값에 필터를 씌우는 함수
void rightCameraFilter(void);   // 오른쪽카메라에서 받아온 값에 필터를 씌우는 함수
void delay(void);            // 카메라를 동작시키기 위해서 딜레이를 발생시키는 함수
int detectLineFromLeftCam();       // 왼쪽카메라에서 받아온 값에서 라인이 있는가를 판단하는 함수
int detectLineFromRightCam();       // 오른쪽카메라에서 받아온 값에서 라인이 있는가를 판단하는 함수
void verySimpleFilterL(int line);   // 배리심플 병우필터 성공
void verySimpleFilterR(int line);   // 배리심플 병우필터 성공
int changeServoAngle(); //edge검출 결과로 servo회전
void BalanceMotor(unsigned long servoValue, unsigned long motorSpeed); //좌 우 바퀴속도를  모터 기본속도와, servo입력값에 따라 바꿈
void SaveAngleByCheck(unsigned long input); //angle이 급격히 변해서 해당 변화를 신용하지 못해 과거 angle을 유지시킨다.
void SensorModule(void); 		 // 적외선 센서와 관련한 모든 함수들이 실행되는 함수
void InitializeSensorData(void); // 적외선 센서의 값을 받아오는 배열을 초기화하는 함수
void EnqueueSensorData(void); 	 // 적외선 센서에 값을 받아오는 함수
int  CheckTrap(void);			 // 장애물이 있는지 없는지를 파악하는 함수, 있는 경우 1, 없는 경우 0을 반환
void DequeueSensorData(void);	 // 다음 적외선 센서를 받기 위하여 dequeue작업 진행
void InitializeSaveLineIndex();
void EnqueueLineIndex(void); 	 // 라인 인덱스 저장
void DequeueLineIndex(void); 	 // 라인 인덱스 딜리트
void EnqueueSchoolCount(int left , int right); 	 // School count 저장
void DequeueSchoolCount(void); 	 // School count 딜리트
void BalanceMotorSZ(unsigned long servoValue); //servo입력값에 따라 스쿨존 속도를 바꿈
int detectSchoolZone(int min1, int min2); //school zone을 찾아냄
/*********************** Define Values******************************/
//servo angle
#define ANGLE_RIGHT_4 47600
#define ANGLE_RIGHT_3 47800
#define ANGLE_RIGHT_2_5 48000
#define ANGLE_RIGHT_2 48200
#define ANGLE_RIGHT_1 48400
#define ANGLE_MID     48600
#define ANGLE_LEFT_1 48800
#define ANGLE_LEFT_2 49000
#define ANGLE_LEFT_2_5 49200
#define ANGLE_LEFT_3 49400
#define ANGLE_LEFT_4 49600
//motor speed
#define MOTOR_MAX_SPEED 100
#define MOTOR_FAST_3 108
#define MOTOR_FAST_2 108
#define MOTOR_FAST_1 114
#define MOTOR_MID    120
#define MOTOR_SLOW_1 126
#define MOTOR_SLOW_2 132
#define MOTOR_SLOW_3 138
#define MOTOR_STOP   170

#define MOTOR_REVERSE_SLOW_1 190
#define MOTOR_REVERSE_SLOW_2 180
#define MOTOR_REVERSE_SLOW_3 170

//회전할때 바퀴마다 속도 차이주려는 rate
#define ROTATION_RATE_1 1.18
#define ROTATION_RATE_2 1.43
#define ROTATION_RATE_3 1.77

//바퀴 속도 차등적용
#define SPEED_RATE_4 60
#define SPEED_RATE_3 40
#define SPEED_RATE_2 20
#define SPEED_RATE_1 10

#define MOTOR_SCHOOLZONE 188

/*************** Global values ************************************///
//camera array
uint16_t CameraArrLeft[128];
uint16_t CameraArrRight[128];
uint16_t CameraArrTemp[128]; //계산용 array

int startFlag;    //0: 정지 , 1: 스타트

int leftCameraLineIndex; //왼쪽 카메라 라인 체크시 그 라인의 index
int rightCameraLineIndex; //왼쪽 카메라 라인 체크시 그 라인의 index
int savedLeftLineIndex[5]; 
int savedRightLineIndex[5];
unsigned long savedServoAngle; //servo angle저장용.
unsigned long savedMotorSpeed; //motor speed저장용.

int savedLeftSchoolCount[10];
int savedRightSchoolCount[10];

//회전할때 바퀴마다 속도 차이주려는 rate
#define ROTATION_RATE_1 1.18
#define ROTATION_RATE_2 1.43
#define ROTATION_RATE_3 1.77


/*************** Global values ************************************///
//camera array
uint16_t CameraArrLeft[128];
uint16_t CameraArrRight[128];
uint16_t CameraArrTemp[128]; //계산용 array

//적외선 센서 array
uint16_t sensorQueue1[5];
uint16_t ckTrap;  //장애물 검사 된 flag value 0: 장애물 x ,  1: 장애물 o
uint16_t szTrap;  //school zone flag value 0:스쿨존 x, 1: 스쿨존 o
uint16_t szIO;    //szIO flag value 0: 스쿨존 out, 1: 스쿨존 in

int leftCameraLineIndex; //왼쪽 카메라 라인 체크시 그 라인의 index
int rightCameraLineIndex; //왼쪽 카메라 라인 체크시 그 라인의 index

unsigned long savedServoAngle; //servo angle저장용.
unsigned long savedMotorSpeed; //motor speed저장용.

int schoolZoneCheck;
int SpinEndCheck; // 회전 끝나고 보정해줄 check변수
int avgSensorValue;
/*********************  Initialization Function(s) ************************/

	void main(void) {
	int i=0;
	int min1; //라인 디텍트 결과 min
	int min2;
	int carActFlag = 0; //DC모터를 제어하는 플래그
	int sensorActFlag = 0;//적외선 센서를 제어하는 플래그	
	int speed=0; //0.2초마다 encoder speed 갱신
	int wheelCheck=0;
	SpinEndCheck =0;
	avgSensorValue = 0;
	//int wheelRotate; //총 회전횟수 누적

	leftCameraLineIndex = -2; //안보인상황
	rightCameraLineIndex = -2;
	savedServoAngle = ANGLE_MID;  //최초의 저장된 servoAngle
	savedMotorSpeed = MOTOR_MID;  //최초 저장된 모터속도
	ckTrap = 0;//장애물을 찾았는가를 체크하는 변수

	szTrap = 0;//스쿨존을 찾았는가를 체크하는 변수
	szIO = 0; //스쿨존의 안인지 밖인지 체크하는 변수
	schoolZoneCheck=0;
	/* ----------------------------------------------------------- */
	/*                System Initialization Function                  */
	/* ----------------------------------------------------------- */
	sys_init_fnc();
	/********* Enable External Interrupt *********/

	EnableExternalInterrupts();

	//***********************  car moving activated      ***********************//
	GPIO_SetState(15, 0);                        // 모터 Off
	InitializeSensorData();                     //적외선 센서부분 초기화
	EMIOS_0 .CH[0].CADR.R = savedServoAngle;    //최초 바퀴 위치 mid
	EMIOS_0 .CH[2].CADR.R = savedMotorSpeed;
	EMIOS_0 .CH[3].CADR.R = 256;
	EMIOS_0 .CH[4].CADR.R = savedMotorSpeed;
	EMIOS_0 .CH[5].CADR.R = 256;

	while (1) {
		speed = Get_Speed();

		if(GPIO_GetState(64)==1) //switch1 on 시 carActFlag변화
		{
			if( carActFlag == 1)
				carActFlag = 0;                       // 모터 off
			else
				carActFlag =1;                        // 모터 on

			if( carActFlag == 1)
				GPIO_SetState(15, 1);                        // 모터 On
			else
				GPIO_SetState(15, 0);                        // 모터 Off

			GPIO_SetState(64, 0);
		}
		//스위치가 눌린경우
		if(GPIO_GetState(65)==1)
		{
			sensorActFlag=1;	//센서를 사용할 수 있도록 플래그 설정
		}

		if( carActFlag == 1 && ckTrap==0){ //장애물이 없으면
			getCamera();                              //카메라 값 받아오기
			leftCameraFilter();                        //왼쪽 카메라에서 받아온 값 필터씌우기
			rightCameraFilter();                        //오른쪽 카메라에서 받아온 값 필터씌우기
			min1 = detectLineFromLeftCam();      //라인을 찾았는지 확인하는 함수, 찾은 경우 최소값의 인덱스반환
			min2 = detectLineFromRightCam();      //라인을 찾았는지 확인하는 함수, 찾은 경우 최소값의 인덱반환
			detectSchoolZone(min1, min2);
			verySimpleFilterL(min1);                     //병우필터 돌았다
			verySimpleFilterR(min2);		
			if(sensorActFlag == 1)
				SensorModule(); 						//센서와 관련된 함수들을 실행한다.   //ckTrap 을 flag로 넘겨줘 장애물 확인
			changeServoAngle();                         //servo회전

			if(szTrap == 0 && szIO == 0)
				BalanceMotor(savedServoAngle, savedMotorSpeed);		
			else if( szTrap == 0 && szIO == 1)
			{
				//	BalanceMotorSZ(savedServoAngle);
				EMIOS_0 .CH[2].CADR.R = MOTOR_SCHOOLZONE;
				EMIOS_0 .CH[3].CADR.R = 256;
				EMIOS_0 .CH[4].CADR.R = MOTOR_SCHOOLZONE;
				EMIOS_0 .CH[5].CADR.R = 256;
			}
			else if( szTrap == 1 && szIO == 0)
			{			
				EMIOS_0 .CH[2].CADR.R = 240;
				EMIOS_0 .CH[3].CADR.R = 256;
				EMIOS_0 .CH[4].CADR.R = 240;
				EMIOS_0 .CH[5].CADR.R = 256;

				if(wheelCheck == 0) {
					Set_wheelRotate(0);
					wheelCheck = 1;
				}
				if(Get_wheelRotate() >= 2) {
					szTrap = 0;
					szIO = 1;
					wheelCheck = 0;
				}
			}
			else if( szTrap == 1 && szIO == 1)
			{
				EMIOS_0 .CH[2].CADR.R = MOTOR_SCHOOLZONE;
				EMIOS_0 .CH[3].CADR.R = 256;
				EMIOS_0 .CH[4].CADR.R = MOTOR_SCHOOLZONE;
				EMIOS_0 .CH[5].CADR.R = 256;

				if(wheelCheck == 0) {
					Set_wheelRotate(0);	
					wheelCheck = 1;
				}
				if(Get_wheelRotate() >= 2) {
					szTrap = 0;
					szIO = 0;
					wheelCheck = 0;
				}
			}			
		}
		else if(carActFlag==1 && ckTrap==1)
		{
			EnqueueSensorData();   //맨 처음 두개의 센서의 값을 받아온다.

			if( sensorQueue1[0] >150 ||sensorQueue1[0] < sensorQueue1[1]+10)// 
			{
				EMIOS_0 .CH[2].CADR.R = 256;
				EMIOS_0 .CH[3].CADR.R = 256;
				EMIOS_0 .CH[4].CADR.R = 256;
				EMIOS_0 .CH[5].CADR.R = 256;
				carActFlag=0;
				ckTrap=0;
				//GPIO_SetState(15,0);
			}
			else
			{
				EMIOS_0 .CH[2].CADR.R = 256;
				EMIOS_0 .CH[3].CADR.R = (sensorQueue1[0]/2)-40;
				EMIOS_0 .CH[4].CADR.R = 256;
				EMIOS_0 .CH[5].CADR.R = (sensorQueue1[0]/2)-40;
			}

			DequeueSensorData();    //한칸씩 민다

			getCamera();                              //카메라 값 받아오기
			leftCameraFilter();                        //왼쪽 카메라에서 받아온 값 필터씌우기
			rightCameraFilter();                        //오른쪽 카메라에서 받아온 값 필터씌우기
			min1 = detectLineFromLeftCam();      //라인을 찾았는지 확인하는 함수, 찾은 경우 최소값의 인덱스반환
			min2 = detectLineFromRightCam();      //라인을 찾았는지 확인하는 함수, 찾은 경우 최소값의 인덱반환
			// detectSchoolZone(min1, min2);
			//FindSchoolZone();                // 병우 스쿨존라인 찾는 함수
			verySimpleFilterL(min1);                     //병우필터 돌았다
			verySimpleFilterR(min2);		
			changeServoAngle();
		}
		else //키면 카메라+서보만 회전  녀석
		{
			getCamera();                              //카메라 값 받아오기
			leftCameraFilter();                        //왼쪽 카메라에서 받아온 값 필터씌우기
			rightCameraFilter();                        //오른쪽 카메라에서 받아온 값 필터씌우기
			min1 = detectLineFromLeftCam();      //라인을 찾았는지 확인하는 함수, 찾은 경우 최소값의 인덱스반환
			min2 = detectLineFromRightCam();      //라인을 찾았는지 확인하는 함수, 찾은 경우 최소값의 인덱반환
			// detectSchoolZone(min1, min2);
			//FindSchoolZone();                // 병우 스쿨존라인 찾는 함수
			verySimpleFilterL(min1);                     //병우필터 돌았다
			verySimpleFilterR(min2);		
			changeServoAngle();                         //servo회전
		}
		/*****************************************/

	}
}
int detectSchoolZone(int min1, int min2) {
	int leftCount=0;
	int rightCount=0;
	int i=0;
	int signLeft=0;
	int signRight=0;
	if(min1 != 0 && min2 != 0) {
		for(i=113; i>63; i--)
			if(CameraArrLeft[i] < 80)
				leftCount++;

		for(i=15; i<65; i++)
			if(CameraArrRight[i] < 80)
				rightCount++;

	}
	else
	{
		leftCount =0;
		rightCount =0;
	}
	DequeueSchoolCount();
	EnqueueSchoolCount(leftCount,rightCount);

	for( i = 0 ; i < 10 ; i++)
	{
		if(savedLeftSchoolCount[i]>=30)
		{
			szTrap = 1;
			return;
		}
		if(savedRightSchoolCount[i]>=30)
		{
			szTrap = 1;
			return;
		}
		if(savedLeftSchoolCount[i]>=20)
		{
			signLeft = 1;
		}
		if(savedRightSchoolCount[i]>=20)
		{
			signRight = 1;
		}
	}
	if(signLeft == 1 && signRight == 1)
	{
		szTrap = 1;
	}

	/*
	int i=0;	if(min1 != 0 && min2 != 0) {
		for(i=113; i>63; i--)
			if(CameraArrLeft[i] < 80)
				leftCount++;

		for(i=15; i<65; i++)
			if(CameraArrRight[i] < 80)
				rightCount++;

		if(leftCount >= 20 || rightCount >= 20)
			szTrap = 1;
	}
	 */
}

void BalanceMotorSZ(unsigned long servoValue) {
	if (servoValue == ANGLE_LEFT_4) {
		EMIOS_0 .CH[2].CADR.R = MOTOR_SCHOOLZONE - 8;
		EMIOS_0 .CH[4].CADR.R = MOTOR_SCHOOLZONE;
		return;
	} else if (servoValue == ANGLE_RIGHT_4) {
		EMIOS_0 .CH[2].CADR.R = MOTOR_SCHOOLZONE;
		EMIOS_0 .CH[4].CADR.R = MOTOR_SCHOOLZONE - 8;
		return;
	}

	if (servoValue >= ANGLE_LEFT_3) {
		EMIOS_0 .CH[2].CADR.R = MOTOR_SCHOOLZONE - 6;
		EMIOS_0 .CH[4].CADR.R = MOTOR_SCHOOLZONE;
		return;
	} else if (servoValue <= ANGLE_RIGHT_3) {
		EMIOS_0 .CH[2].CADR.R = MOTOR_SCHOOLZONE;
		EMIOS_0 .CH[4].CADR.R = MOTOR_SCHOOLZONE - 6;
		return;
	} else if (servoValue >= ANGLE_LEFT_2) {
		EMIOS_0 .CH[2].CADR.R = MOTOR_SCHOOLZONE - 4;
		EMIOS_0 .CH[4].CADR.R = MOTOR_SCHOOLZONE;
		return;
	} else if (servoValue <= ANGLE_RIGHT_2) {
		EMIOS_0 .CH[2].CADR.R = MOTOR_SCHOOLZONE;
		EMIOS_0 .CH[4].CADR.R = MOTOR_SCHOOLZONE - 4;
		return;
	}

	else if (servoValue >= ANGLE_LEFT_1) {
		EMIOS_0 .CH[2].CADR.R = MOTOR_SCHOOLZONE - 2;
		EMIOS_0 .CH[4].CADR.R = MOTOR_SCHOOLZONE;
		return;
	} else if (servoValue <= ANGLE_RIGHT_1) {
		EMIOS_0 .CH[2].CADR.R = MOTOR_SCHOOLZONE;
		EMIOS_0 .CH[4].CADR.R = MOTOR_SCHOOLZONE - 2;
		return;
	} else {
		EMIOS_0 .CH[2].CADR.R = MOTOR_SCHOOLZONE;
		EMIOS_0 .CH[4].CADR.R = MOTOR_SCHOOLZONE;
		return;
	}
}

void SaveAngleByCheck(unsigned long newAngle) {
	//방향전환 차이값이 너무큰 상황. 코스감지를 이상하게 잃었다고 의심.
	int i;
	int j;
	i = (int)newAngle - (int)savedServoAngle;
	j = (int)savedServoAngle - (int)newAngle;

	if (i <= 1200) {
		if (j <= 1200) {
			savedServoAngle = newAngle;
		}
	}
}
void delay(void) {
	int delay_i;
	delay_i = 100;
	while (delay_i) {
		delay_i--;
	}
}

void getCamera(void) {
	int camera_i;

	if (savedServoAngle < ANGLE_MID) //우회전중이면 우측 라인카메라 우선순위가 높아야함. (트랙벗어나는거 방지)
	{
		GPIO_SetState(108, 1); // camera 2 si high
		GPIO_SetState(28, 1); // camera 1 si high
		delay();
		GPIO_SetState(109, 1); // camera 2 clk high
		GPIO_SetState(29, 1); // camera 1 clk high
		delay();
		GPIO_SetState(108, 0); // camera 2 si low
		GPIO_SetState(28, 0); // camera 1 si low
		CameraArrRight[0] = (uint16_t) A2D_GetSingleCh_10bit(1);
		CameraArrLeft[0] = (uint16_t) A2D_GetSingleCh_10bit(0);
		delay();
		GPIO_SetState(109, 0); // camera 2 clk high

		GPIO_SetState(29, 0); // camera 1 clk low

		for (camera_i = 1; camera_i < 128; camera_i++) {
			delay();
			delay();
			GPIO_SetState(109, 1); // camera 2 clk high
			GPIO_SetState(29, 1); // camera 1 clk high 
			delay();
			delay();
			CameraArrRight[camera_i] = (uint16_t) A2D_GetSingleCh_10bit(1);
			CameraArrLeft[camera_i] = (uint16_t) A2D_GetSingleCh_10bit(0);
			GPIO_SetState(109, 0); // camera 2 clk low
			GPIO_SetState(29, 0); // camera 1 clk low
		}

		delay();
		delay();
		GPIO_SetState(109, 1); // camera 2 clk high
		GPIO_SetState(29, 1); // camera 1 clk high
		delay();
		delay();
		GPIO_SetState(109, 0); // camera 2 clk low
		GPIO_SetState(29, 0); // camera 1 clk low
		delay();
		delay();
	} else //좌회전중이니 좌카메라의 우선순위가 높음.
	{

		GPIO_SetState(28, 1); // camera 1 si high
		GPIO_SetState(108, 1); // camera 2 si high
		delay();
		GPIO_SetState(29, 1); // camera 1 clk high
		GPIO_SetState(109, 1); // camera 2 clk high
		delay();
		GPIO_SetState(28, 0); // camera 1 si low
		GPIO_SetState(108, 0); // camera 2 si low
		CameraArrLeft[0] = (uint16_t) A2D_GetSingleCh_10bit(0);
		CameraArrRight[0] = (uint16_t) A2D_GetSingleCh_10bit(1);
		delay();
		GPIO_SetState(29, 0); // camera 1 clk low
		GPIO_SetState(109, 0); // camera 2 clk high

		for (camera_i = 1; camera_i < 128; camera_i++) {
			delay();
			delay();
			GPIO_SetState(29, 1); // camera 1 clk high 
			GPIO_SetState(109, 1); // camera 2 clk high
			delay();
			delay();
			CameraArrLeft[camera_i] = (uint16_t) A2D_GetSingleCh_10bit(0);
			CameraArrRight[camera_i] = (uint16_t) A2D_GetSingleCh_10bit(1);
			GPIO_SetState(29, 0); // camera 1 clk low
			GPIO_SetState(109, 0); // camera 2 clk low
		}

		delay();
		delay();
		GPIO_SetState(29, 1); // camera 1 clk high
		GPIO_SetState(109, 1); // camera 2 clk high
		delay();
		delay();
		GPIO_SetState(29, 0); // camera 1 clk low
		GPIO_SetState(109, 0); // camera 2 clk low
		delay();
		delay();

	}
}

void leftCameraFilter(void) {
	int i, j;
	int sum; //카메라 값을 모두 더한 값
	for (i = 0; i < 15; i++)
		CameraArrLeft[i] = 0;
	for (i = 114; i < 128; i++)
		CameraArrLeft[i] = 0;
	sum = 0;
	for (i = 0; i < 128; i++) {
		CameraArrTemp[i] = CameraArrLeft[i]; //cop해놓음
	}
	for (i = 17; i <= 111; i++) {
		//5개의 픽셀 값을 모두 더한다
		for (j = i - 2; j <= i + 2; j++) {
			sum += CameraArrTemp[j];
		}
		CameraArrLeft[i] = (uint16_t) (sum / 5);
		sum = 0;
	}

	CameraArrLeft[15] = CameraArrLeft[17]; //최고극단 은 최단거리 안쪽 값으로 배정
	CameraArrLeft[16] = CameraArrLeft[17];
	CameraArrLeft[112] = CameraArrLeft[111];
	CameraArrLeft[113] = CameraArrLeft[111];
}

void rightCameraFilter(void) {
	int i, j;
	int sum; //카메라 값을 모두 더한 값

	for (i = 0; i < 15; i++)
		CameraArrRight[i] = 0;
	for (i = 114; i < 128; i++)
		CameraArrRight[i] = 0;

	sum = 0;
	for (i = 0; i < 128; i++) {
		CameraArrTemp[i] = CameraArrRight[i]; //cop해놓음
	}

	for (i = 17; i <= 111; i++) {
		//5개의 픽셀 값을 모두 더한다
		for (j = i - 2; j <= i + 2; j++) {
			sum += CameraArrTemp[j];
		}
		CameraArrRight[i] = (uint16_t) (sum / 5); //평균값
		sum = 0;
	}
	CameraArrRight[15] = CameraArrRight[17]; //최고극단 은 최단거리 안쪽 값으로 배정
	CameraArrRight[16] = CameraArrRight[17];
	CameraArrRight[112] = CameraArrRight[111];
	CameraArrRight[113] = CameraArrRight[111];
}

int detectLineFromLeftCam() {
	int max = 0;   // 최대값 저장하는 변수
	int maxIndex;      // 최대값의 위치
	int min = 10000; // 최소값 저장하는 변수
	int minIndex;
	int i;
	int isLine = 0; //선이 있는가를 확인하는 변수

	//07/03 재성 이 for( 0~ 127 을   127 ~> 0으로 진행하게 변경
	for (i = 127; i >= 0; i--) {
		// 현재 카메라 값이 max값보다 큰 경우
		if (CameraArrLeft[i] > max) {
			max = CameraArrLeft[i];
			maxIndex = i; // 현재 값 저장
		}
		//현재 최소값보다 값이 작은경우
		if ((CameraArrLeft[i] < (max - 50)) && (CameraArrLeft[i] < min)) {
			min = CameraArrLeft[i];
			minIndex = i; // 현재값 저장
		}
		//현재 최소값보다 50 큰 값이 나올때
		if ((CameraArrLeft[i] > min + 50)) {
			isLine = minIndex; //최소값의 위치를 반환
			return isLine; //값 반환
		}
	}
	return isLine; //값 반환
}

void verySimpleFilterL(int line) {
	int i;
	for (i = 15; i < 114; i++) {
		CameraArrLeft[i] = 150; //모든 값을 150으로 만든다.
	}
	//라인을 찾은경우
	if (line != 0) {
		CameraArrLeft[line] = 1;
		leftCameraLineIndex = line; //라인 index저장
		return;
	}
	leftCameraLineIndex = -2; //라인이 없다.
	return;
}

int detectLineFromRightCam() {
	int max = 0;   // 최대값 저장하는 변수
	int maxIndex;      // 최대값의 위치
	int min = 10000; // 최소값 저장하는 변수
	int minIndex;
	int i;
	int isLine = 0; //선이 있는가를 확인하는 변수

	for (i = 0; i < 128; i++) {
		// 현재 카메라 값이 max값보다 큰 경우
		if (CameraArrRight[i] > max) {
			max = CameraArrRight[i];
			maxIndex = i; // 현재 값 저장
		}
		//현재 최소값보다 값이 작은경우
		if ((CameraArrRight[i] < (max - 50)) && (CameraArrRight[i] < min)) {
			min = CameraArrRight[i];
			minIndex = i; // 현재값 저장
		}
		//현재 최소값보다 50 큰 값이 나올때
		if ((CameraArrRight[i] > min + 50)) {
			isLine = minIndex; //최소값의 위치를 반환
			return isLine; //값 반환
		}
	}
	return isLine; //값 반환
}

void verySimpleFilterR(int line) {
	int i;
	for (i = 15; i < 114; i++) {
		CameraArrRight[i] = 150; //모든 값을 150으로 만든다.
	}
	//라인을 찾은경우
	if (line != 0) {
		CameraArrRight[line] = 1;
		rightCameraLineIndex = line; //라인 index저장
		return;
	}

	//라인이 없는 경우
	rightCameraLineIndex = -2; //라인 index저장 
	return;

}
int changeServoAngle() {

	//edgeDetection할필요없이 line index 범위가지고 결정하자.
	//index변환 범위 : 0xxxxxx15<= ~ <35<=~  <49<= ~ <79<= ~ <94<= ~ <114xxxxx 이런 방식으로 5구간으로 나눔

	DequeueLineIndex(); 	 // 라인 인덱스 딜리트
	EnqueueLineIndex(); 	 // 라인 인덱스 저장

	//해당 leftCameraLineIndex,rightCameraLineIndex값을 바탕으로 servo 각도 변경
	if (leftCameraLineIndex == -2) {
		if (rightCameraLineIndex == -2) //       {         |         } 상태
		{

			if (savedServoAngle <= ANGLE_LEFT_1
					&& savedServoAngle >= ANGLE_RIGHT_1) {
				//servo 제어
				SaveAngleByCheck(ANGLE_MID);
				EMIOS_0 .CH[0].CADR.R = savedServoAngle;

				savedMotorSpeed = MOTOR_SLOW_1;
				//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
				return;

			} else if (savedServoAngle >= ANGLE_LEFT_2
					&& savedServoAngle < ANGLE_LEFT_4) {
				//왼쪽으로 꺽다가 벗어날 위기에 빠진상황
				if (savedServoAngle + 100 <= ANGLE_LEFT_4) //한계전 까지
					savedServoAngle = savedServoAngle + 100; //순차적인 증가

				//if( savedMotorSpeed <= MOTOR_SLOW_3 -6)
				if( savedMotorSpeed >= MOTOR_FAST_2 +6)
					savedMotorSpeed = savedMotorSpeed - 6; //순차적 증가

				EMIOS_0 .CH[0].CADR.R = savedServoAngle;
				if(savedServoAngle>= ANGLE_LEFT_2_5  ||savedServoAngle<= ANGLE_RIGHT_2_5)
				{
					SpinEndCheck = 30;
				}
				//BalanceMotor(savedServoAngle, savedMotorSpeed); //motor control
				return;
			}

			else if (savedServoAngle <= ANGLE_RIGHT_2
					&& savedServoAngle < ANGLE_RIGHT_4) {
				//오른쪽으로 꺽다가 벗어날 위기에 빠진상황
				if (savedServoAngle - 100 >= ANGLE_RIGHT_4) //한계전 까지
					savedServoAngle = savedServoAngle - 100; //순차적 감소

				//if( savedMotorSpeed <= MOTOR_SLOW_3 -6)
				if( savedMotorSpeed >= MOTOR_FAST_2 +6)
					savedMotorSpeed = savedMotorSpeed - 6; //순차적 증가
				EMIOS_0 .CH[0].CADR.R = savedServoAngle;
				if(savedServoAngle>= ANGLE_LEFT_2_5  ||savedServoAngle<= ANGLE_RIGHT_2_5)
				{
					SpinEndCheck = 30;
				}
				//BalanceMotor(savedServoAngle, savedMotorSpeed); //motor control
				return;
			} else {
				EMIOS_0 .CH[0].CADR.R = savedServoAngle;
				//BalanceMotor(savedServoAngle, savedMotorSpeed); //motor control
				if(savedServoAngle>= ANGLE_LEFT_2_5  ||savedServoAngle<= ANGLE_RIGHT_2_5)
				{
					SpinEndCheck = 30;
				}
				return;
			}

		} 

		/*
		else if (rightCameraLineIndex < 35) //       {         |*        } 상태
		{

			SaveAngleByCheck(ANGLE_LEFT_4);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_MID); //motor control
			return;
		} else if (rightCameraLineIndex < 49) //       {         |  *      } 상태
		{
			SaveAngleByCheck(ANGLE_LEFT_3);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_1;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_1); //motor control
			return;

		} else if (rightCameraLineIndex < 79) //       {         |     *    } 상태
		{
			SaveAngleByCheck(ANGLE_LEFT_2);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_2;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_2); //motor control
			return;

		} else if (rightCameraLineIndex < 94) //       {         |       *  } 상태
		{
			SaveAngleByCheck(ANGLE_LEFT_2);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_2;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_2); //motor control
			return;

		} else if (rightCameraLineIndex < 114) //       {         |         *} 상태
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_3;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_3); //motor control
			return;

		}
		 */
		else
		{


			SaveAngleByCheck(ANGLE_MID + (113-rightCameraLineIndex)*8);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			if(rightCameraLineIndex>= 64)
			{
				savedMotorSpeed =MOTOR_SLOW_2;
			}
			else
			{
				savedMotorSpeed = MOTOR_MID + ((int)(113-rightCameraLineIndex))/6;
			}
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_3); //motor control
			if(savedServoAngle>= ANGLE_LEFT_2_5  ||savedServoAngle<= ANGLE_RIGHT_2_5)
			{
				SpinEndCheck = 30;
			}
			return;


		}
	} else if (leftCameraLineIndex < 35) {
		if (rightCameraLineIndex == -2) //       { *        |         } 상태
		{
			/*
			SaveAngleByCheck(ANGLE_RIGHT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_3;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_3); //motor control
			return;
			 */
			SaveAngleByCheck(ANGLE_MID - (leftCameraLineIndex-15)*8);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;

			if(leftCameraLineIndex< 64)
			{
				savedMotorSpeed =MOTOR_SLOW_2;
			}
			else
			{
				savedMotorSpeed = MOTOR_MID + ((int)(leftCameraLineIndex-15))/6;
			}
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_3); //motor control
			if(savedServoAngle>= ANGLE_LEFT_2_5  ||savedServoAngle<= ANGLE_RIGHT_2_5)
			{
				SpinEndCheck = 30;
			}
			return;


		} else if (rightCameraLineIndex < 35) //       { *        |*        } 상태
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_1;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_1); //motor control
			return;

		} else if (rightCameraLineIndex < 49) //       { *        |  *        } 상태
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_MID); //motor control
			return;

		} else if (rightCameraLineIndex < 79) //       { *        |     *    } 상태
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_1;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_1); //motor control
			return;

		} else if (rightCameraLineIndex < 94) //       { *        |       *  } 상태
		{
			SaveAngleByCheck(ANGLE_MID - (rightCameraLineIndex-79)*5  + (35- leftCameraLineIndex)*5);

			//SaveAngleByCheck(ANGLE_MID);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
			return;

		} else if (rightCameraLineIndex < 114) //       { *        |        * } 상태
		{
			SaveAngleByCheck(ANGLE_MID);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_2;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
			return;

		}
	} else if (leftCameraLineIndex < 49) {
		if (rightCameraLineIndex == -2) //       {   *       |         } 상태
		{
			/*
			SaveAngleByCheck(ANGLE_RIGHT_2);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_2;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_2); //motor control
			return;
			 */
			SaveAngleByCheck(ANGLE_MID - (leftCameraLineIndex-15)*8);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			if(leftCameraLineIndex< 64)
			{
				savedMotorSpeed =MOTOR_SLOW_2;
			}
			else
			{
				savedMotorSpeed = MOTOR_MID + ((int)(leftCameraLineIndex-15))/6;
			}
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_3); //motor control
			if(savedServoAngle>= ANGLE_LEFT_2_5  ||savedServoAngle<= ANGLE_RIGHT_2_5)
			{
				SpinEndCheck = 30;
			}
			return;
		} else if (rightCameraLineIndex < 35) //       {   *      | *        } 상태
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_1;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_1); //motor control
			return;

		} else if (rightCameraLineIndex < 49) //       {   *      |   *       } 상태
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_1;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_1); //motor control
			return;

		} else if (rightCameraLineIndex < 79) //       {   *      |     *    } 상태
		{
			//SaveAngleByCheck(ANGLE_MID);
			SaveAngleByCheck(ANGLE_MID - (rightCameraLineIndex-49)*3  + (49- leftCameraLineIndex)*10);

			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
			return;

		} else if (rightCameraLineIndex < 94) //       {   *      |       *  } 상태
		{
			SaveAngleByCheck(ANGLE_MID);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_2;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
			return;

		} else if (rightCameraLineIndex < 114) //       {   *      |        * } 상태
		{
			SaveAngleByCheck(ANGLE_MID - (rightCameraLineIndex-94)*5  + (49- leftCameraLineIndex)*5);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
			return;

		}
	} else if (leftCameraLineIndex < 79) {
		if (rightCameraLineIndex == -2) //       {    *     |         } 상태
		{
			/*
			SaveAngleByCheck(ANGLE_RIGHT_2);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_2;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_2); //motor control
			return;
			 */
			SaveAngleByCheck(ANGLE_MID - (leftCameraLineIndex-15)*8);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			if(leftCameraLineIndex< 64)
			{
				savedMotorSpeed =MOTOR_SLOW_2;
			}
			else
			{
				savedMotorSpeed = MOTOR_MID + ((int)(leftCameraLineIndex-15))/6;
			}
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_3); //motor control
			if(savedServoAngle>= ANGLE_LEFT_2_5  ||savedServoAngle<= ANGLE_RIGHT_2_5)
			{
				SpinEndCheck = 30;
			}
			return;

		} else if (rightCameraLineIndex < 35) //       {    *     |*        } 상태
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_2;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_2); //motor control
			return;

		} else if (rightCameraLineIndex < 49) //       {    *     |  *        } 상태
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_1;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_1); //motor control
			return;

		} else if (rightCameraLineIndex < 79) //       {    *     |     *    } 상태
		{
			SaveAngleByCheck(ANGLE_MID);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_2;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
			return;

		} else if (rightCameraLineIndex < 94) //       {    *     |        * } 상태
		{

			SaveAngleByCheck(ANGLE_MID - (94-rightCameraLineIndex)*10 + (79-leftCameraLineIndex)*3 );
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
			return;

		} else if (rightCameraLineIndex < 114) //       {    *     |          *} 상태
		{
			SaveAngleByCheck(ANGLE_RIGHT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_1;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_1); //motor control
			return;

		}
	}

	else if (leftCameraLineIndex < 94) {
		if (rightCameraLineIndex == -2) //       {       *  |         } 상태
		{

			/*
			SaveAngleByCheck(ANGLE_RIGHT_3);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_1;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_1); //motor control
			return;
			 */
			SaveAngleByCheck(ANGLE_MID - (leftCameraLineIndex-15)*8);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			if(leftCameraLineIndex< 64)
			{
				savedMotorSpeed =MOTOR_SLOW_2;
			}
			else
			{
				savedMotorSpeed = MOTOR_MID + ((int)(leftCameraLineIndex-15))/6;
			}
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_3); //motor control
			if(savedServoAngle>= ANGLE_LEFT_2_5  ||savedServoAngle<= ANGLE_RIGHT_2_5)
			{
				SpinEndCheck = 30;
			}
			return;
		} else if (rightCameraLineIndex < 49) //       {      *  | * *       } 상태
		{
			//이상한 상황이니 이전상태대로 느리게 진행
			savedMotorSpeed = MOTOR_SLOW_2;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_2); //motor control
			return;

		} else if (rightCameraLineIndex < 79) //       {      *  |     *    } 상태
		{
			SaveAngleByCheck(ANGLE_RIGHT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_1;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_1); //motor control
			return;

		} else if (rightCameraLineIndex < 94) //       {      *  |       *  } 상태
		{
			SaveAngleByCheck(ANGLE_RIGHT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_1;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_1); //motor control
			return;
		} else if (rightCameraLineIndex < 114) //       {      *  |         *} 상태
		{
			SaveAngleByCheck(ANGLE_RIGHT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_MID); //motor control
			return;
		}
	} else if (leftCameraLineIndex < 114) {
		if (rightCameraLineIndex == -2) //       {        * |         } 상태
		{
			/*SaveAngleByCheck(ANGLE_RIGHT_4);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_MID); //motor control
			return;
			 */
			SaveAngleByCheck(ANGLE_MID - (leftCameraLineIndex-15)*8);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			if(leftCameraLineIndex< 64)
			{
				savedMotorSpeed =MOTOR_SLOW_2;
			}
			else
			{
				savedMotorSpeed = MOTOR_MID + ((int)(leftCameraLineIndex-15))/6;
			}
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_3); //motor control
			if(savedServoAngle>= ANGLE_LEFT_2_5  ||savedServoAngle<= ANGLE_RIGHT_2_5)
			{
				SpinEndCheck = 30;
			}
			return;
		} else if (rightCameraLineIndex < 49) //       {       * | * *       } 상태
		{
			//이상한 상황이니 이전 servo대로 이동
			savedMotorSpeed = MOTOR_SLOW_2;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_2); //motor control
			return;

		} else if (rightCameraLineIndex < 79) //       {       * |     *    } 상태
		{
			SaveAngleByCheck((unsigned long) ANGLE_RIGHT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_2;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_2); //motor control
			return;

		} else if (rightCameraLineIndex < 94) //       {        * |       *  } 상태
		{
			SaveAngleByCheck((unsigned long) ANGLE_RIGHT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_1;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_1); //motor control
			return;
		} else if (rightCameraLineIndex < 114) //       {        * |         *} 상태
		{
			SaveAngleByCheck(ANGLE_RIGHT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_1;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_1); //motor control
			return;
		}
	}

}

void BalanceMotor(unsigned long servoValue, unsigned long motorSpeed) 
{
	//szTrap부분으로 스쿨존 진입시 정지
	if( savedLeftLineIndex[0]!= -2 &&savedRightLineIndex[0]!= -2) //인라인 라인내부에 있으려고 반대바퀴에 힘을줌
	{
		/*
		if( SpinEndCheck>15) //감속후 출발할 필요성이 있다.
		{
			SpinEndCheck--;
			EMIOS_0 .CH[2].CADR.R = 256;
			EMIOS_0 .CH[3].CADR.R = 220- (20-SpinEndCheck)*16;
			EMIOS_0 .CH[4].CADR.R = 256;
			EMIOS_0 .CH[5].CADR.R = 220- (20-SpinEndCheck)*16;
			savedMotorSpeed = motorSpeed;
			return;
		}else if( SpinEndCheck>5)
		{

			SpinEndCheck--;
			EMIOS_0 .CH[2].CADR.R = 256;
			EMIOS_0 .CH[3].CADR.R = 140;
			EMIOS_0 .CH[4].CADR.R = 256;
			EMIOS_0 .CH[5].CADR.R = 140;
			savedMotorSpeed = motorSpeed;
			return;
		}else if( SpinEndCheck>0)
		{

			SpinEndCheck--;
			EMIOS_0 .CH[2].CADR.R = 256;
			EMIOS_0 .CH[3].CADR.R = 220 - (SpinEndCheck*16);
			EMIOS_0 .CH[4].CADR.R = 256;
			EMIOS_0 .CH[5].CADR.R = 220 - (SpinEndCheck*16);
			savedMotorSpeed = motorSpeed;
			return;
		}

		 */
		if( SpinEndCheck>0)
		{

			SpinEndCheck--;
			EMIOS_0 .CH[2].CADR.R = 255;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = 255;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		}
		else
		{
			if (servoValue <= ANGLE_LEFT_4) {

				EMIOS_0 .CH[2].CADR.R = motorSpeed + SPEED_RATE_4;
				EMIOS_0 .CH[3].CADR.R = 256;
				EMIOS_0 .CH[4].CADR.R = motorSpeed;
				EMIOS_0 .CH[5].CADR.R = 256;
				savedMotorSpeed = motorSpeed;
				return;
			} else if (servoValue >= ANGLE_RIGHT_4) {
				EMIOS_0 .CH[2].CADR.R = motorSpeed;
				EMIOS_0 .CH[3].CADR.R = 256;
				EMIOS_0 .CH[4].CADR.R = motorSpeed + SPEED_RATE_4;
				EMIOS_0 .CH[5].CADR.R = 256;
				savedMotorSpeed = motorSpeed;
				return;
			}

			else if (servoValue >= ANGLE_LEFT_3) {
				EMIOS_0 .CH[2].CADR.R = motorSpeed+ SPEED_RATE_3;
				EMIOS_0 .CH[3].CADR.R = 256;
				EMIOS_0 .CH[4].CADR.R = motorSpeed ;
				EMIOS_0 .CH[5].CADR.R = 256;
				savedMotorSpeed = motorSpeed;
				return;
			} else if (servoValue <= ANGLE_RIGHT_3) {
				EMIOS_0 .CH[2].CADR.R = motorSpeed;
				EMIOS_0 .CH[3].CADR.R = 256;
				EMIOS_0 .CH[4].CADR.R = motorSpeed+ SPEED_RATE_3;
				EMIOS_0 .CH[5].CADR.R = 256;
				savedMotorSpeed = motorSpeed;
				return;
			}  else if (servoValue >= ANGLE_LEFT_2) {
				EMIOS_0 .CH[2].CADR.R = motorSpeed + SPEED_RATE_3;
				EMIOS_0 .CH[3].CADR.R = 256;
				EMIOS_0 .CH[4].CADR.R = motorSpeed;
				EMIOS_0 .CH[5].CADR.R = 256;
				savedMotorSpeed = motorSpeed;
				return;
			} else if (servoValue <= ANGLE_RIGHT_2) {
				EMIOS_0 .CH[2].CADR.R = motorSpeed ;
				EMIOS_0 .CH[3].CADR.R = 256;
				EMIOS_0 .CH[4].CADR.R = motorSpeed+ SPEED_RATE_3;
				EMIOS_0 .CH[5].CADR.R = 256;
				savedMotorSpeed = motorSpeed;
				return;
			}

			else if (servoValue >= ANGLE_LEFT_1) {
				EMIOS_0 .CH[2].CADR.R = motorSpeed+ SPEED_RATE_2;
				EMIOS_0 .CH[3].CADR.R = 256;
				EMIOS_0 .CH[4].CADR.R =  motorSpeed;
				EMIOS_0 .CH[5].CADR.R = 256;
				savedMotorSpeed = motorSpeed;
				return;
			} else if (servoValue <= ANGLE_RIGHT_1) {
				EMIOS_0 .CH[2].CADR.R =  motorSpeed;
				EMIOS_0 .CH[3].CADR.R = 256;
				EMIOS_0 .CH[4].CADR.R =  motorSpeed+ SPEED_RATE_2;
				EMIOS_0 .CH[5].CADR.R = 256;
				savedMotorSpeed = motorSpeed;
				return;
			} else {
				EMIOS_0 .CH[2].CADR.R = motorSpeed;
				EMIOS_0 .CH[3].CADR.R = 256;
				EMIOS_0 .CH[4].CADR.R = motorSpeed;
				EMIOS_0 .CH[5].CADR.R = 256;
				savedMotorSpeed = motorSpeed;
				return;
			}	

		}


	}
	
	/*
	else if( savedLeftLineIndex[0]== -2 &&savedRightLineIndex[0]!= -2 && savedLeftLineIndex[4]!= -2 &&savedRightLineIndex[4]!= -2) //이전까진 라인보이다가 왼쪽만 안보이는 상황
	{
		if (servoValue <= ANGLE_LEFT_4) {

			EMIOS_0 .CH[2].CADR.R = motorSpeed;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed + SPEED_RATE_4;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		} else if (servoValue >= ANGLE_RIGHT_4) {
			EMIOS_0 .CH[2].CADR.R = motorSpeed + SPEED_RATE_4;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		}

		else if (servoValue >= ANGLE_LEFT_3) {
			EMIOS_0 .CH[2].CADR.R = motorSpeed;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed + SPEED_RATE_3;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		} else if (servoValue <= ANGLE_RIGHT_3) {
			EMIOS_0 .CH[2].CADR.R = motorSpeed + SPEED_RATE_3;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		}  else if (servoValue >= ANGLE_LEFT_2) {

			EMIOS_0 .CH[2].CADR.R = motorSpeed;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = 256;
			EMIOS_0 .CH[5].CADR.R = MOTOR_STOP- SPEED_RATE_3;
			savedMotorSpeed = motorSpeed;
			return;
		} else if (servoValue <= ANGLE_RIGHT_2) {
			EMIOS_0 .CH[2].CADR.R = 256;
			EMIOS_0 .CH[3].CADR.R = MOTOR_STOP- SPEED_RATE_3;
			EMIOS_0 .CH[4].CADR.R = motorSpeed;
			EMIOS_0 .CH[5].CADR.R = 256;
			return;
		}

		else if (servoValue >= ANGLE_LEFT_1) {
			EMIOS_0 .CH[2].CADR.R = 256;
			EMIOS_0 .CH[3].CADR.R = MOTOR_STOP-SPEED_RATE_2;
			EMIOS_0 .CH[4].CADR.R = 256;
			EMIOS_0 .CH[5].CADR.R = MOTOR_STOP- SPEED_RATE_4;
			savedMotorSpeed = motorSpeed;
			return;
		} else if (servoValue <= ANGLE_RIGHT_1) {
			EMIOS_0 .CH[2].CADR.R = 256;
			EMIOS_0 .CH[3].CADR.R = MOTOR_STOP- SPEED_RATE_4;
			EMIOS_0 .CH[4].CADR.R = 256;
			EMIOS_0 .CH[5].CADR.R = MOTOR_STOP-SPEED_RATE_2;
			savedMotorSpeed = motorSpeed;
			return;
		} else {
			EMIOS_0 .CH[2].CADR.R = motorSpeed;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		}	


	}
	else if( savedLeftLineIndex[0]!= -2 &&savedRightLineIndex[0]== -2 && savedLeftLineIndex[4]!= -2 &&savedRightLineIndex[4]!= -2) 
		//이전까진 라인보이다가 오른쪽만 안보이는 상황
	{
		if (servoValue >= ANGLE_LEFT_4) {

			EMIOS_0 .CH[2].CADR.R = motorSpeed;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed + SPEED_RATE_4;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		} else if (servoValue <= ANGLE_RIGHT_4) {
			EMIOS_0 .CH[2].CADR.R = motorSpeed + SPEED_RATE_4;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		}

		else if (servoValue >= ANGLE_LEFT_3) {
			EMIOS_0 .CH[2].CADR.R = motorSpeed;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed + SPEED_RATE_3;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		} else if (servoValue <= ANGLE_RIGHT_3) {
			EMIOS_0 .CH[2].CADR.R = motorSpeed + SPEED_RATE_3;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		} else if (servoValue >= ANGLE_LEFT_2) {

			EMIOS_0 .CH[2].CADR.R = motorSpeed;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = 256;
			EMIOS_0 .CH[5].CADR.R = MOTOR_STOP- SPEED_RATE_3;
			savedMotorSpeed = motorSpeed;
			return;
		} else if (servoValue <= ANGLE_RIGHT_2) {
			EMIOS_0 .CH[2].CADR.R = 256;
			EMIOS_0 .CH[3].CADR.R = MOTOR_STOP- SPEED_RATE_3;
			EMIOS_0 .CH[4].CADR.R = motorSpeed;
			EMIOS_0 .CH[5].CADR.R = 256;
			return;
		}

		else if (servoValue >= ANGLE_LEFT_1) {
			EMIOS_0 .CH[2].CADR.R = 256;
			EMIOS_0 .CH[3].CADR.R = MOTOR_STOP-SPEED_RATE_2;
			EMIOS_0 .CH[4].CADR.R = 256;
			EMIOS_0 .CH[5].CADR.R = MOTOR_STOP- SPEED_RATE_4;
			savedMotorSpeed = motorSpeed;
			return;
		} else if (servoValue <= ANGLE_RIGHT_1) {
			EMIOS_0 .CH[2].CADR.R = 256;
			EMIOS_0 .CH[3].CADR.R = MOTOR_STOP- SPEED_RATE_4;
			EMIOS_0 .CH[4].CADR.R = 256;
			EMIOS_0 .CH[5].CADR.R = MOTOR_STOP-SPEED_RATE_2;
			savedMotorSpeed = motorSpeed;
			return;
		} else {
			EMIOS_0 .CH[2].CADR.R = motorSpeed;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		}	

	}
	
	
	*/
	else{

		if (servoValue == ANGLE_LEFT_4) {

			EMIOS_0 .CH[2].CADR.R = motorSpeed;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed + SPEED_RATE_4;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		} else if (servoValue == ANGLE_RIGHT_4) {
			EMIOS_0 .CH[2].CADR.R = motorSpeed + SPEED_RATE_4;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		}

		if (servoValue >= ANGLE_LEFT_3) {
			EMIOS_0 .CH[2].CADR.R = motorSpeed;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed + SPEED_RATE_3;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		} else if (servoValue <= ANGLE_RIGHT_3) {
			EMIOS_0 .CH[2].CADR.R = motorSpeed + SPEED_RATE_3;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		} else if (servoValue >= ANGLE_LEFT_2) {
			EMIOS_0 .CH[2].CADR.R = motorSpeed;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed + SPEED_RATE_2;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		} else if (servoValue <= ANGLE_RIGHT_2) {
			EMIOS_0 .CH[2].CADR.R = motorSpeed + SPEED_RATE_2;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		}

		else if (servoValue >= ANGLE_LEFT_1) {
			EMIOS_0 .CH[2].CADR.R = motorSpeed;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed + SPEED_RATE_1;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		} else if (servoValue <= ANGLE_RIGHT_1) {
			EMIOS_0 .CH[2].CADR.R = motorSpeed + SPEED_RATE_1;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		} else {
			EMIOS_0 .CH[2].CADR.R = motorSpeed;
			EMIOS_0 .CH[3].CADR.R = 256;
			EMIOS_0 .CH[4].CADR.R = motorSpeed;
			EMIOS_0 .CH[5].CADR.R = 256;
			savedMotorSpeed = motorSpeed;
			return;
		}

	}
}


void SensorModule(void)
{
	int check;			//장애물이 있는가를 판단하기 위한 변수
	EnqueueSensorData();	//맨 처음 두개의 센서의 값을 받아온다.
	check = CheckTrap();//장애물이 있는가를 판단하는 함수를 실행시키고 값을 받아온다. 장애물이 있는 경우 1, 없는 경우 0을 반환
	//장애물이 없는 경우
	if(check==0)
	{
		DequeueSensorData();
	}	
	//장애물을 찾은 경우
	else if(check==1)
	{
		ckTrap=1;//장애물을 찾았음을 메인함수에 알린다
		//속도 감소시키는 값 입력
	}
	else
	{
		//에러
	}
}

void InitializeSensorData(void)
{
	int i;
	//값을 초기화한다.
	for(i=0;i<5;i++)
	{
		sensorQueue1[i] = 0;
	}
}

void EnqueueSensorData(void)
{
	//ADC를 이용하여 각각의 적외선 센서에서 값을 받아온다.
	sensorQueue1[0] = A2D_GetSingleCh_10bit(2);
}
void EnqueueLineIndex(void)
{
	savedLeftLineIndex[0] = leftCameraLineIndex;
	savedRightLineIndex[0] = rightCameraLineIndex;
}

void EnqueueSchoolCount(int left , int right) 	 // School count 저장
{
	savedLeftSchoolCount[0] = left;
	savedRightSchoolCount[0] = right;

}

void DequeueSchoolCount(void) 	 // School count 딜리트
{
	int i;
	//값을 하나씩 뒤로 이동시킨다.
	for(i=8;i>=0;i--)
	{
		savedLeftSchoolCount[i+1] = savedLeftSchoolCount[i];
		savedRightSchoolCount[i+1] = savedRightSchoolCount[i];
	}
}

void DequeueLineIndex(void)
{
	int i;
	//값을 하나씩 뒤로 이동시킨다.
	for(i=3;i>=0;i--)
	{
		savedLeftLineIndex[i+1] = savedLeftLineIndex[i];
		savedRightLineIndex[i+1] = savedRightLineIndex[i];
	}
}

int CheckTrap(void)
{
	int avg;
	if(sensorQueue1[4]!=0)
	{
		avg=(sensorQueue1[0]+sensorQueue1[1]+sensorQueue1[2])/3;

		if(avgSensorValue==0)
			avgSensorValue = avg;
		
		else
		{
			if(avg>avgSensorValue+50)
				return 1;//장애물감지
		}
	}

	return 0;//장애물을 감지하지 못함
	/*
	int i;
	int count1, count2;
	count1 = 0;
	count2 = 0;
	for(i=0;i<4;i++)
	{
		//다음 배열에 값이 남아있는 경우
		if(sensorQueue1[i+1]!=0 )
		{
			if(sensorQueue1[i+1]>120 && sensorQueue1[i]>120)
			{
				count1++;
			}
		}
	}
	if(count1==4)
	{
		return 1;//장애물감지
	}
	else
	{
		return 0;//장애물을 감지하지 못함

	}*/

}

void DequeueSensorData(void)
{
	int i;
	//값을 하나씩 뒤로 이동시킨다.
	for(i=3;i>=0;i--)
	{
		sensorQueue1[i+1] = sensorQueue1[i];
	}
}


void InitializeSaveLineIndex()
{
	int i;

	for(i=0 ; i<5 ; i++)
	{
		savedLeftLineIndex[i] = 64;
		savedRightLineIndex[i] = 64;

	}

}


/*
 *######################################################################
 *                           End of File
 *######################################################################
 */

