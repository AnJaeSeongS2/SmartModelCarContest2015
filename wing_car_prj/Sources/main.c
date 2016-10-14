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
void getCamera(void);         // ī�޶� ���� �޾ƿ��� ���� �Լ�
void leftCameraFilter(void);   // ����ī�޶󿡼� �޾ƿ� ���� ���͸� ����� �Լ�
void rightCameraFilter(void);   // ������ī�޶󿡼� �޾ƿ� ���� ���͸� ����� �Լ�
void delay(void);            // ī�޶� ���۽�Ű�� ���ؼ� �����̸� �߻���Ű�� �Լ�
int detectLineFromLeftCam();       // ����ī�޶󿡼� �޾ƿ� ������ ������ �ִ°��� �Ǵ��ϴ� �Լ�
int detectLineFromRightCam();       // ������ī�޶󿡼� �޾ƿ� ������ ������ �ִ°��� �Ǵ��ϴ� �Լ�
void verySimpleFilterL(int line);   // �踮���� �������� ����
void verySimpleFilterR(int line);   // �踮���� �������� ����
int changeServoAngle(); //edge���� ����� servoȸ��
void BalanceMotor(unsigned long servoValue, unsigned long motorSpeed); //�� �� �����ӵ���  ���� �⺻�ӵ���, servo�Է°��� ���� �ٲ�
void SaveAngleByCheck(unsigned long input); //angle�� �ް��� ���ؼ� �ش� ��ȭ�� �ſ����� ���� ���� angle�� ������Ų��.
void SensorModule(void); 		 // ���ܼ� ������ ������ ��� �Լ����� ����Ǵ� �Լ�
void InitializeSensorData(void); // ���ܼ� ������ ���� �޾ƿ��� �迭�� �ʱ�ȭ�ϴ� �Լ�
void EnqueueSensorData(void); 	 // ���ܼ� ������ ���� �޾ƿ��� �Լ�
int  CheckTrap(void);			 // ��ֹ��� �ִ��� �������� �ľ��ϴ� �Լ�, �ִ� ��� 1, ���� ��� 0�� ��ȯ
void DequeueSensorData(void);	 // ���� ���ܼ� ������ �ޱ� ���Ͽ� dequeue�۾� ����
void InitializeSaveLineIndex();
void EnqueueLineIndex(void); 	 // ���� �ε��� ����
void DequeueLineIndex(void); 	 // ���� �ε��� ����Ʈ
void EnqueueSchoolCount(int left , int right); 	 // School count ����
void DequeueSchoolCount(void); 	 // School count ����Ʈ
void BalanceMotorSZ(unsigned long servoValue); //servo�Է°��� ���� ������ �ӵ��� �ٲ�
int detectSchoolZone(int min1, int min2); //school zone�� ã�Ƴ�
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

//ȸ���Ҷ� �������� �ӵ� �����ַ��� rate
#define ROTATION_RATE_1 1.18
#define ROTATION_RATE_2 1.43
#define ROTATION_RATE_3 1.77

//���� �ӵ� ��������
#define SPEED_RATE_4 60
#define SPEED_RATE_3 40
#define SPEED_RATE_2 20
#define SPEED_RATE_1 10

#define MOTOR_SCHOOLZONE 188

/*************** Global values ************************************///
//camera array
uint16_t CameraArrLeft[128];
uint16_t CameraArrRight[128];
uint16_t CameraArrTemp[128]; //���� array

int startFlag;    //0: ���� , 1: ��ŸƮ

int leftCameraLineIndex; //���� ī�޶� ���� üũ�� �� ������ index
int rightCameraLineIndex; //���� ī�޶� ���� üũ�� �� ������ index
int savedLeftLineIndex[5]; 
int savedRightLineIndex[5];
unsigned long savedServoAngle; //servo angle�����.
unsigned long savedMotorSpeed; //motor speed�����.

int savedLeftSchoolCount[10];
int savedRightSchoolCount[10];

//ȸ���Ҷ� �������� �ӵ� �����ַ��� rate
#define ROTATION_RATE_1 1.18
#define ROTATION_RATE_2 1.43
#define ROTATION_RATE_3 1.77


/*************** Global values ************************************///
//camera array
uint16_t CameraArrLeft[128];
uint16_t CameraArrRight[128];
uint16_t CameraArrTemp[128]; //���� array

//���ܼ� ���� array
uint16_t sensorQueue1[5];
uint16_t ckTrap;  //��ֹ� �˻� �� flag value 0: ��ֹ� x ,  1: ��ֹ� o
uint16_t szTrap;  //school zone flag value 0:������ x, 1: ������ o
uint16_t szIO;    //szIO flag value 0: ������ out, 1: ������ in

int leftCameraLineIndex; //���� ī�޶� ���� üũ�� �� ������ index
int rightCameraLineIndex; //���� ī�޶� ���� üũ�� �� ������ index

unsigned long savedServoAngle; //servo angle�����.
unsigned long savedMotorSpeed; //motor speed�����.

int schoolZoneCheck;
int SpinEndCheck; // ȸ�� ������ �������� check����
int avgSensorValue;
/*********************  Initialization Function(s) ************************/

	void main(void) {
	int i=0;
	int min1; //���� ����Ʈ ��� min
	int min2;
	int carActFlag = 0; //DC���͸� �����ϴ� �÷���
	int sensorActFlag = 0;//���ܼ� ������ �����ϴ� �÷���	
	int speed=0; //0.2�ʸ��� encoder speed ����
	int wheelCheck=0;
	SpinEndCheck =0;
	avgSensorValue = 0;
	//int wheelRotate; //�� ȸ��Ƚ�� ����

	leftCameraLineIndex = -2; //�Ⱥ��λ�Ȳ
	rightCameraLineIndex = -2;
	savedServoAngle = ANGLE_MID;  //������ ����� servoAngle
	savedMotorSpeed = MOTOR_MID;  //���� ����� ���ͼӵ�
	ckTrap = 0;//��ֹ��� ã�Ҵ°��� üũ�ϴ� ����

	szTrap = 0;//�������� ã�Ҵ°��� üũ�ϴ� ����
	szIO = 0; //�������� ������ ������ üũ�ϴ� ����
	schoolZoneCheck=0;
	/* ----------------------------------------------------------- */
	/*                System Initialization Function                  */
	/* ----------------------------------------------------------- */
	sys_init_fnc();
	/********* Enable External Interrupt *********/

	EnableExternalInterrupts();

	//***********************  car moving activated      ***********************//
	GPIO_SetState(15, 0);                        // ���� Off
	InitializeSensorData();                     //���ܼ� �����κ� �ʱ�ȭ
	EMIOS_0 .CH[0].CADR.R = savedServoAngle;    //���� ���� ��ġ mid
	EMIOS_0 .CH[2].CADR.R = savedMotorSpeed;
	EMIOS_0 .CH[3].CADR.R = 256;
	EMIOS_0 .CH[4].CADR.R = savedMotorSpeed;
	EMIOS_0 .CH[5].CADR.R = 256;

	while (1) {
		speed = Get_Speed();

		if(GPIO_GetState(64)==1) //switch1 on �� carActFlag��ȭ
		{
			if( carActFlag == 1)
				carActFlag = 0;                       // ���� off
			else
				carActFlag =1;                        // ���� on

			if( carActFlag == 1)
				GPIO_SetState(15, 1);                        // ���� On
			else
				GPIO_SetState(15, 0);                        // ���� Off

			GPIO_SetState(64, 0);
		}
		//����ġ�� �������
		if(GPIO_GetState(65)==1)
		{
			sensorActFlag=1;	//������ ����� �� �ֵ��� �÷��� ����
		}

		if( carActFlag == 1 && ckTrap==0){ //��ֹ��� ������
			getCamera();                              //ī�޶� �� �޾ƿ���
			leftCameraFilter();                        //���� ī�޶󿡼� �޾ƿ� �� ���;����
			rightCameraFilter();                        //������ ī�޶󿡼� �޾ƿ� �� ���;����
			min1 = detectLineFromLeftCam();      //������ ã�Ҵ��� Ȯ���ϴ� �Լ�, ã�� ��� �ּҰ��� �ε�����ȯ
			min2 = detectLineFromRightCam();      //������ ã�Ҵ��� Ȯ���ϴ� �Լ�, ã�� ��� �ּҰ��� �ε���ȯ
			detectSchoolZone(min1, min2);
			verySimpleFilterL(min1);                     //�������� ���Ҵ�
			verySimpleFilterR(min2);		
			if(sensorActFlag == 1)
				SensorModule(); 						//������ ���õ� �Լ����� �����Ѵ�.   //ckTrap �� flag�� �Ѱ��� ��ֹ� Ȯ��
			changeServoAngle();                         //servoȸ��

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
			EnqueueSensorData();   //�� ó�� �ΰ��� ������ ���� �޾ƿ´�.

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

			DequeueSensorData();    //��ĭ�� �δ�

			getCamera();                              //ī�޶� �� �޾ƿ���
			leftCameraFilter();                        //���� ī�޶󿡼� �޾ƿ� �� ���;����
			rightCameraFilter();                        //������ ī�޶󿡼� �޾ƿ� �� ���;����
			min1 = detectLineFromLeftCam();      //������ ã�Ҵ��� Ȯ���ϴ� �Լ�, ã�� ��� �ּҰ��� �ε�����ȯ
			min2 = detectLineFromRightCam();      //������ ã�Ҵ��� Ȯ���ϴ� �Լ�, ã�� ��� �ּҰ��� �ε���ȯ
			// detectSchoolZone(min1, min2);
			//FindSchoolZone();                // ���� ���������� ã�� �Լ�
			verySimpleFilterL(min1);                     //�������� ���Ҵ�
			verySimpleFilterR(min2);		
			changeServoAngle();
		}
		else //Ű�� ī�޶�+������ ȸ��  �༮
		{
			getCamera();                              //ī�޶� �� �޾ƿ���
			leftCameraFilter();                        //���� ī�޶󿡼� �޾ƿ� �� ���;����
			rightCameraFilter();                        //������ ī�޶󿡼� �޾ƿ� �� ���;����
			min1 = detectLineFromLeftCam();      //������ ã�Ҵ��� Ȯ���ϴ� �Լ�, ã�� ��� �ּҰ��� �ε�����ȯ
			min2 = detectLineFromRightCam();      //������ ã�Ҵ��� Ȯ���ϴ� �Լ�, ã�� ��� �ּҰ��� �ε���ȯ
			// detectSchoolZone(min1, min2);
			//FindSchoolZone();                // ���� ���������� ã�� �Լ�
			verySimpleFilterL(min1);                     //�������� ���Ҵ�
			verySimpleFilterR(min2);		
			changeServoAngle();                         //servoȸ��
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
	//������ȯ ���̰��� �ʹ�ū ��Ȳ. �ڽ������� �̻��ϰ� �Ҿ��ٰ� �ǽ�.
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

	if (savedServoAngle < ANGLE_MID) //��ȸ�����̸� ���� ����ī�޶� �켱������ ���ƾ���. (Ʈ������°� ����)
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
	} else //��ȸ�����̴� ��ī�޶��� �켱������ ����.
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
	int sum; //ī�޶� ���� ��� ���� ��
	for (i = 0; i < 15; i++)
		CameraArrLeft[i] = 0;
	for (i = 114; i < 128; i++)
		CameraArrLeft[i] = 0;
	sum = 0;
	for (i = 0; i < 128; i++) {
		CameraArrTemp[i] = CameraArrLeft[i]; //cop�س���
	}
	for (i = 17; i <= 111; i++) {
		//5���� �ȼ� ���� ��� ���Ѵ�
		for (j = i - 2; j <= i + 2; j++) {
			sum += CameraArrTemp[j];
		}
		CameraArrLeft[i] = (uint16_t) (sum / 5);
		sum = 0;
	}

	CameraArrLeft[15] = CameraArrLeft[17]; //�ְ�ش� �� �ִܰŸ� ���� ������ ����
	CameraArrLeft[16] = CameraArrLeft[17];
	CameraArrLeft[112] = CameraArrLeft[111];
	CameraArrLeft[113] = CameraArrLeft[111];
}

void rightCameraFilter(void) {
	int i, j;
	int sum; //ī�޶� ���� ��� ���� ��

	for (i = 0; i < 15; i++)
		CameraArrRight[i] = 0;
	for (i = 114; i < 128; i++)
		CameraArrRight[i] = 0;

	sum = 0;
	for (i = 0; i < 128; i++) {
		CameraArrTemp[i] = CameraArrRight[i]; //cop�س���
	}

	for (i = 17; i <= 111; i++) {
		//5���� �ȼ� ���� ��� ���Ѵ�
		for (j = i - 2; j <= i + 2; j++) {
			sum += CameraArrTemp[j];
		}
		CameraArrRight[i] = (uint16_t) (sum / 5); //��հ�
		sum = 0;
	}
	CameraArrRight[15] = CameraArrRight[17]; //�ְ�ش� �� �ִܰŸ� ���� ������ ����
	CameraArrRight[16] = CameraArrRight[17];
	CameraArrRight[112] = CameraArrRight[111];
	CameraArrRight[113] = CameraArrRight[111];
}

int detectLineFromLeftCam() {
	int max = 0;   // �ִ밪 �����ϴ� ����
	int maxIndex;      // �ִ밪�� ��ġ
	int min = 10000; // �ּҰ� �����ϴ� ����
	int minIndex;
	int i;
	int isLine = 0; //���� �ִ°��� Ȯ���ϴ� ����

	//07/03 �缺 �� for( 0~ 127 ��   127 ~> 0���� �����ϰ� ����
	for (i = 127; i >= 0; i--) {
		// ���� ī�޶� ���� max������ ū ���
		if (CameraArrLeft[i] > max) {
			max = CameraArrLeft[i];
			maxIndex = i; // ���� �� ����
		}
		//���� �ּҰ����� ���� �������
		if ((CameraArrLeft[i] < (max - 50)) && (CameraArrLeft[i] < min)) {
			min = CameraArrLeft[i];
			minIndex = i; // ���簪 ����
		}
		//���� �ּҰ����� 50 ū ���� ���ö�
		if ((CameraArrLeft[i] > min + 50)) {
			isLine = minIndex; //�ּҰ��� ��ġ�� ��ȯ
			return isLine; //�� ��ȯ
		}
	}
	return isLine; //�� ��ȯ
}

void verySimpleFilterL(int line) {
	int i;
	for (i = 15; i < 114; i++) {
		CameraArrLeft[i] = 150; //��� ���� 150���� �����.
	}
	//������ ã�����
	if (line != 0) {
		CameraArrLeft[line] = 1;
		leftCameraLineIndex = line; //���� index����
		return;
	}
	leftCameraLineIndex = -2; //������ ����.
	return;
}

int detectLineFromRightCam() {
	int max = 0;   // �ִ밪 �����ϴ� ����
	int maxIndex;      // �ִ밪�� ��ġ
	int min = 10000; // �ּҰ� �����ϴ� ����
	int minIndex;
	int i;
	int isLine = 0; //���� �ִ°��� Ȯ���ϴ� ����

	for (i = 0; i < 128; i++) {
		// ���� ī�޶� ���� max������ ū ���
		if (CameraArrRight[i] > max) {
			max = CameraArrRight[i];
			maxIndex = i; // ���� �� ����
		}
		//���� �ּҰ����� ���� �������
		if ((CameraArrRight[i] < (max - 50)) && (CameraArrRight[i] < min)) {
			min = CameraArrRight[i];
			minIndex = i; // ���簪 ����
		}
		//���� �ּҰ����� 50 ū ���� ���ö�
		if ((CameraArrRight[i] > min + 50)) {
			isLine = minIndex; //�ּҰ��� ��ġ�� ��ȯ
			return isLine; //�� ��ȯ
		}
	}
	return isLine; //�� ��ȯ
}

void verySimpleFilterR(int line) {
	int i;
	for (i = 15; i < 114; i++) {
		CameraArrRight[i] = 150; //��� ���� 150���� �����.
	}
	//������ ã�����
	if (line != 0) {
		CameraArrRight[line] = 1;
		rightCameraLineIndex = line; //���� index����
		return;
	}

	//������ ���� ���
	rightCameraLineIndex = -2; //���� index���� 
	return;

}
int changeServoAngle() {

	//edgeDetection���ʿ���� line index ���������� ��������.
	//index��ȯ ���� : 0xxxxxx15<= ~ <35<=~  <49<= ~ <79<= ~ <94<= ~ <114xxxxx �̷� ������� 5�������� ����

	DequeueLineIndex(); 	 // ���� �ε��� ����Ʈ
	EnqueueLineIndex(); 	 // ���� �ε��� ����

	//�ش� leftCameraLineIndex,rightCameraLineIndex���� �������� servo ���� ����
	if (leftCameraLineIndex == -2) {
		if (rightCameraLineIndex == -2) //       {         |         } ����
		{

			if (savedServoAngle <= ANGLE_LEFT_1
					&& savedServoAngle >= ANGLE_RIGHT_1) {
				//servo ����
				SaveAngleByCheck(ANGLE_MID);
				EMIOS_0 .CH[0].CADR.R = savedServoAngle;

				savedMotorSpeed = MOTOR_SLOW_1;
				//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
				return;

			} else if (savedServoAngle >= ANGLE_LEFT_2
					&& savedServoAngle < ANGLE_LEFT_4) {
				//�������� ���ٰ� ��� ���⿡ ������Ȳ
				if (savedServoAngle + 100 <= ANGLE_LEFT_4) //�Ѱ��� ����
					savedServoAngle = savedServoAngle + 100; //�������� ����

				//if( savedMotorSpeed <= MOTOR_SLOW_3 -6)
				if( savedMotorSpeed >= MOTOR_FAST_2 +6)
					savedMotorSpeed = savedMotorSpeed - 6; //������ ����

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
				//���������� ���ٰ� ��� ���⿡ ������Ȳ
				if (savedServoAngle - 100 >= ANGLE_RIGHT_4) //�Ѱ��� ����
					savedServoAngle = savedServoAngle - 100; //������ ����

				//if( savedMotorSpeed <= MOTOR_SLOW_3 -6)
				if( savedMotorSpeed >= MOTOR_FAST_2 +6)
					savedMotorSpeed = savedMotorSpeed - 6; //������ ����
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
		else if (rightCameraLineIndex < 35) //       {         |*        } ����
		{

			SaveAngleByCheck(ANGLE_LEFT_4);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_MID); //motor control
			return;
		} else if (rightCameraLineIndex < 49) //       {         |  *      } ����
		{
			SaveAngleByCheck(ANGLE_LEFT_3);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_1;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_1); //motor control
			return;

		} else if (rightCameraLineIndex < 79) //       {         |     *    } ����
		{
			SaveAngleByCheck(ANGLE_LEFT_2);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_2;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_2); //motor control
			return;

		} else if (rightCameraLineIndex < 94) //       {         |       *  } ����
		{
			SaveAngleByCheck(ANGLE_LEFT_2);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_2;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_2); //motor control
			return;

		} else if (rightCameraLineIndex < 114) //       {         |         *} ����
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
		if (rightCameraLineIndex == -2) //       { *        |         } ����
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


		} else if (rightCameraLineIndex < 35) //       { *        |*        } ����
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_1;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_1); //motor control
			return;

		} else if (rightCameraLineIndex < 49) //       { *        |  *        } ����
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_MID); //motor control
			return;

		} else if (rightCameraLineIndex < 79) //       { *        |     *    } ����
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_1;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_1); //motor control
			return;

		} else if (rightCameraLineIndex < 94) //       { *        |       *  } ����
		{
			SaveAngleByCheck(ANGLE_MID - (rightCameraLineIndex-79)*5  + (35- leftCameraLineIndex)*5);

			//SaveAngleByCheck(ANGLE_MID);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
			return;

		} else if (rightCameraLineIndex < 114) //       { *        |        * } ����
		{
			SaveAngleByCheck(ANGLE_MID);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_2;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
			return;

		}
	} else if (leftCameraLineIndex < 49) {
		if (rightCameraLineIndex == -2) //       {   *       |         } ����
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
		} else if (rightCameraLineIndex < 35) //       {   *      | *        } ����
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_1;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_1); //motor control
			return;

		} else if (rightCameraLineIndex < 49) //       {   *      |   *       } ����
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_1;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_1); //motor control
			return;

		} else if (rightCameraLineIndex < 79) //       {   *      |     *    } ����
		{
			//SaveAngleByCheck(ANGLE_MID);
			SaveAngleByCheck(ANGLE_MID - (rightCameraLineIndex-49)*3  + (49- leftCameraLineIndex)*10);

			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
			return;

		} else if (rightCameraLineIndex < 94) //       {   *      |       *  } ����
		{
			SaveAngleByCheck(ANGLE_MID);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_2;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
			return;

		} else if (rightCameraLineIndex < 114) //       {   *      |        * } ����
		{
			SaveAngleByCheck(ANGLE_MID - (rightCameraLineIndex-94)*5  + (49- leftCameraLineIndex)*5);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
			return;

		}
	} else if (leftCameraLineIndex < 79) {
		if (rightCameraLineIndex == -2) //       {    *     |         } ����
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

		} else if (rightCameraLineIndex < 35) //       {    *     |*        } ����
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_2;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_2); //motor control
			return;

		} else if (rightCameraLineIndex < 49) //       {    *     |  *        } ����
		{
			SaveAngleByCheck(ANGLE_LEFT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_1;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_1); //motor control
			return;

		} else if (rightCameraLineIndex < 79) //       {    *     |     *    } ����
		{
			SaveAngleByCheck(ANGLE_MID);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_2;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
			return;

		} else if (rightCameraLineIndex < 94) //       {    *     |        * } ����
		{

			SaveAngleByCheck(ANGLE_MID - (94-rightCameraLineIndex)*10 + (79-leftCameraLineIndex)*3 );
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_2); //motor control
			return;

		} else if (rightCameraLineIndex < 114) //       {    *     |          *} ����
		{
			SaveAngleByCheck(ANGLE_RIGHT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_1;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_1); //motor control
			return;

		}
	}

	else if (leftCameraLineIndex < 94) {
		if (rightCameraLineIndex == -2) //       {       *  |         } ����
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
		} else if (rightCameraLineIndex < 49) //       {      *  | * *       } ����
		{
			//�̻��� ��Ȳ�̴� �������´�� ������ ����
			savedMotorSpeed = MOTOR_SLOW_2;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_2); //motor control
			return;

		} else if (rightCameraLineIndex < 79) //       {      *  |     *    } ����
		{
			SaveAngleByCheck(ANGLE_RIGHT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_1;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_1); //motor control
			return;

		} else if (rightCameraLineIndex < 94) //       {      *  |       *  } ����
		{
			SaveAngleByCheck(ANGLE_RIGHT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_FAST_1;
			//BalanceMotor(savedServoAngle, MOTOR_FAST_1); //motor control
			return;
		} else if (rightCameraLineIndex < 114) //       {      *  |         *} ����
		{
			SaveAngleByCheck(ANGLE_RIGHT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_MID;
			//BalanceMotor(savedServoAngle, MOTOR_MID); //motor control
			return;
		}
	} else if (leftCameraLineIndex < 114) {
		if (rightCameraLineIndex == -2) //       {        * |         } ����
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
		} else if (rightCameraLineIndex < 49) //       {       * | * *       } ����
		{
			//�̻��� ��Ȳ�̴� ���� servo��� �̵�
			savedMotorSpeed = MOTOR_SLOW_2;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_2); //motor control
			return;

		} else if (rightCameraLineIndex < 79) //       {       * |     *    } ����
		{
			SaveAngleByCheck((unsigned long) ANGLE_RIGHT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_2;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_2); //motor control
			return;

		} else if (rightCameraLineIndex < 94) //       {        * |       *  } ����
		{
			SaveAngleByCheck((unsigned long) ANGLE_RIGHT_1);
			EMIOS_0 .CH[0].CADR.R = savedServoAngle;
			savedMotorSpeed = MOTOR_SLOW_1;
			//BalanceMotor(savedServoAngle, MOTOR_SLOW_1); //motor control
			return;
		} else if (rightCameraLineIndex < 114) //       {        * |         *} ����
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
	//szTrap�κ����� ������ ���Խ� ����
	if( savedLeftLineIndex[0]!= -2 &&savedRightLineIndex[0]!= -2) //�ζ��� ���γ��ο� �������� �ݴ������ ������
	{
		/*
		if( SpinEndCheck>15) //������ ����� �ʿ伺�� �ִ�.
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
	else if( savedLeftLineIndex[0]== -2 &&savedRightLineIndex[0]!= -2 && savedLeftLineIndex[4]!= -2 &&savedRightLineIndex[4]!= -2) //�������� ���κ��̴ٰ� ���ʸ� �Ⱥ��̴� ��Ȳ
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
		//�������� ���κ��̴ٰ� �����ʸ� �Ⱥ��̴� ��Ȳ
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
	int check;			//��ֹ��� �ִ°��� �Ǵ��ϱ� ���� ����
	EnqueueSensorData();	//�� ó�� �ΰ��� ������ ���� �޾ƿ´�.
	check = CheckTrap();//��ֹ��� �ִ°��� �Ǵ��ϴ� �Լ��� �����Ű�� ���� �޾ƿ´�. ��ֹ��� �ִ� ��� 1, ���� ��� 0�� ��ȯ
	//��ֹ��� ���� ���
	if(check==0)
	{
		DequeueSensorData();
	}	
	//��ֹ��� ã�� ���
	else if(check==1)
	{
		ckTrap=1;//��ֹ��� ã������ �����Լ��� �˸���
		//�ӵ� ���ҽ�Ű�� �� �Է�
	}
	else
	{
		//����
	}
}

void InitializeSensorData(void)
{
	int i;
	//���� �ʱ�ȭ�Ѵ�.
	for(i=0;i<5;i++)
	{
		sensorQueue1[i] = 0;
	}
}

void EnqueueSensorData(void)
{
	//ADC�� �̿��Ͽ� ������ ���ܼ� �������� ���� �޾ƿ´�.
	sensorQueue1[0] = A2D_GetSingleCh_10bit(2);
}
void EnqueueLineIndex(void)
{
	savedLeftLineIndex[0] = leftCameraLineIndex;
	savedRightLineIndex[0] = rightCameraLineIndex;
}

void EnqueueSchoolCount(int left , int right) 	 // School count ����
{
	savedLeftSchoolCount[0] = left;
	savedRightSchoolCount[0] = right;

}

void DequeueSchoolCount(void) 	 // School count ����Ʈ
{
	int i;
	//���� �ϳ��� �ڷ� �̵���Ų��.
	for(i=8;i>=0;i--)
	{
		savedLeftSchoolCount[i+1] = savedLeftSchoolCount[i];
		savedRightSchoolCount[i+1] = savedRightSchoolCount[i];
	}
}

void DequeueLineIndex(void)
{
	int i;
	//���� �ϳ��� �ڷ� �̵���Ų��.
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
				return 1;//��ֹ�����
		}
	}

	return 0;//��ֹ��� �������� ����
	/*
	int i;
	int count1, count2;
	count1 = 0;
	count2 = 0;
	for(i=0;i<4;i++)
	{
		//���� �迭�� ���� �����ִ� ���
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
		return 1;//��ֹ�����
	}
	else
	{
		return 0;//��ֹ��� �������� ����

	}*/

}

void DequeueSensorData(void)
{
	int i;
	//���� �ϳ��� �ڷ� �̵���Ų��.
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

