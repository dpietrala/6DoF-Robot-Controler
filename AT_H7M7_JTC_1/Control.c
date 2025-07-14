#include "Control.h"
sControl Control;
sTrajectory Traj __attribute__((section (".ARM.__at_0x24000000"))); // Trajektoria, w pamieci AXI_RAM, max 512kB
#ifdef DEBUG
sDebug Debug  __attribute__((section (".ARM.__at_0x30000000"))); // Bufory komunikacji debugowania, w pamieci SRAM1 do SRAM3, max 288kB
#endif
#ifdef MATLABSIM
sMatSim	MatSim  __attribute__((section (".ARM.__at_0x30000000"))); // Bufory komunikacji debugowania, w pamieci SRAM1 do SRAM3, max 288kB
#endif
sMB_RTUSlave	Mbs __attribute__((section (".ARM.__at_0x38000000"))); // Bufory komunikacji poprzez ModBus, w pamieci SRAM4, max 64kB
sControl* pC = &Control;

// *********************** General functions ***************************************
static void Control_SetTable(double* t, double a0, double a1, double a2, double a3, double a4, double a5)
{
	t[0] = a0; t[1] = a1; t[2] = a2; t[3] = a3; t[4] = a4; t[5] = a5;
}
void Control_SetDefualtArmModel(void)
{
//	//Zmiana w dniu 2022.09.15 08:30
//	pC->Arm.Joints[0].origin = Vec6SetValues(0, 0, 0.047, 0, 0, 0);										//joint 0 coordinate system
//	pC->Arm.Joints[1].origin = Vec6SetValues(0.063, 0, 0.0739, 0, M_PI_2, 0);					//joint 1 coordinate system
//	pC->Arm.Joints[2].origin = Vec6SetValues(-0.686, 0, 0, 0, M_PI, 0);								//joint 2 coordinate system
//	pC->Arm.Joints[3].origin = Vec6SetValues(0.671, 0, -0.0055, M_PI, 0, 0);					//joint 3 coordinate system
//	pC->Arm.Joints[4].origin = Vec6SetValues(0.0675, 0, 0.048, 0, -M_PI_2, M_PI);			//joint 4 coordinate system
//	pC->Arm.Joints[5].origin = Vec6SetValues(0.0675, 0, 0.048, 0, M_PI_2, 0);					//joint 5 coordinate system
//	pC->Arm.Joints[6].origin = Vec6SetValues(0, 0, 0, 0, 0, 0);												//joint 6 coordinate system
//	
//	pC->Arm.Links[0].origin = Vec6SetValues(0, -0.0129, 0.00034, 0, 0, 0);						//center of mass link 0 coordinate system
//	pC->Arm.Links[1].origin = Vec6SetValues(-0.00005, -0.007, 0.0175, 0, 0, M_PI_2);	//center of mass link 1 coordinate system
//	pC->Arm.Links[2].origin = Vec6SetValues(-0.343, 0, 0.05884, 0, M_PI, M_PI_2);			//center of mass link 2 coordinate system
//	pC->Arm.Links[3].origin = Vec6SetValues(0.38666, 0, 0.051, 0, 0, M_PI_2);					//center of mass link 3 coordinate system
//	pC->Arm.Links[4].origin = Vec6SetValues(0.019, 0, 0.059, 0, 0, M_PI_2);						//center of mass link 4 coordinate system
//	pC->Arm.Links[5].origin = Vec6SetValues(0.019, 0, 0.059, 0, 0, M_PI_2);						//center of mass link 5 coordinate system
//	pC->Arm.Links[6].origin = Vec6SetValues(0, 0, 0, 0, 0, 0);												//center of mass link 6 coordinate system
//	
//	pC->Arm.Links[0].innertia = Vec6SetValues(1, 0, 0, 1, 0, 1);													//link 0 innertial values
//	pC->Arm.Links[1].innertia = Vec6SetValues(0.005372, 0, 0, 0.004691, 0, 0.005059);			//link 1 innertial values
//	pC->Arm.Links[2].innertia = Vec6SetValues(0.8437, 0, 0, 0.01143, 0, 0.8428);					//link 2 innertial values
//	pC->Arm.Links[3].innertia = Vec6SetValues(0.3605, 0, 0, 0.006825, 0, 0.3606);					//link 3 innertial values
//	pC->Arm.Links[4].innertia = Vec6SetValues(0.002233, 0, 0, 0.002047, 0, 0.002384);			//link 4 innertial values
//	pC->Arm.Links[5].innertia = Vec6SetValues(0.002233, 0, 0, 0.002047, 0, 0.002384);			//link 5 innertial values
//	pC->Arm.Links[6].innertia = Vec6SetValues(0.000041, 0, 0, 0.000042, 0, 0.00008);			//link 6 innertial values
//	
//	pC->Arm.Links[0].mass = 0.750;		//mass of link 0
//	pC->Arm.Links[1].mass = 3.800;		//mass of link 1
//	pC->Arm.Links[2].mass = 7.200;		//mass of link 2
//	pC->Arm.Links[3].mass = 4.100;		//mass of link 3
//	pC->Arm.Links[4].mass = 1.960;		//mass of link 4
//	pC->Arm.Links[5].mass = 1.960;		//mass of link 5
//	pC->Arm.Links[6].mass = 0.001;		//mass of link 6
	
	
	pC->Arm.Joints[0].origin = Vec6SetValues(0, 0, 0.047, 0, 0, 0);									//joint 0 coordinate system
	pC->Arm.Joints[1].origin = Vec6SetValues(0.063, 0, 0.0739, 0, M_PI_2, 0);					//joint 1 coordinate system
	pC->Arm.Joints[2].origin = Vec6SetValues(-0.686, 0, 0, 0, M_PI, 0);						//joint 2 coordinate system
	pC->Arm.Joints[3].origin = Vec6SetValues(0.671, 0, -0.0055, M_PI, 0, 0);					//joint 3 coordinate system
	pC->Arm.Joints[4].origin = Vec6SetValues(0.0675, 0, 0.048, 0, -M_PI_2, M_PI);	//joint 4 coordinate system
	pC->Arm.Joints[5].origin = Vec6SetValues(0.0675, 0, 0.048, 0, M_PI_2, 0);					//joint 5 coordinate system
	pC->Arm.Joints[6].origin = Vec6SetValues(0, 0, 0, 0, 0, 0);										//joint 6 coordinate system - selected tool

	pC->Arm.Links[0].origin = Vec6SetValues(0, -0.0129, 0.00034, 0, 0, 0);								//center of mass link 0 coordinate system
	pC->Arm.Links[1].origin = Vec6SetValues(0.0077, -0.00001, 0.05524, 0, 0, 0);						//center of mass link 1 coordinate system
	pC->Arm.Links[2].origin = Vec6SetValues(-0.343, 0, 0.05884, 0, 0,0);								//center of mass link 2 coordinate system
	pC->Arm.Links[3].origin = Vec6SetValues(0.38233, 0, 0.05316, 0, 0, 0);								//center of mass link 3 coordinate system
	pC->Arm.Links[4].origin = Vec6SetValues(0.019, 0, 0.04288, 0, 0, 0);								//center of mass link 4 coordinate system
	pC->Arm.Links[5].origin = Vec6SetValues(0.019, 0, 0.04288, 0, 0, 0);								//center of mass link 5 coordinate system
	pC->Arm.Links[6].origin = Vec6SetValues(0, 0, 0, 0, 0, 0);											//center of mass link 6 coordinate system - selected tool

	pC->Arm.Links[0].innertia = Vec6SetValues(1, 0, 0, 1, 0, 1);										//link 0 innertial values
	pC->Arm.Links[1].innertia = Vec6SetValues(0.004799, 0, 0, 0.005509, 0, 0.0051306);					//link 1 innertial values
	pC->Arm.Links[2].innertia = Vec6SetValues(0.012724, 0, 0, 0.86065, 0, 0.858842);					//link 2 innertial values
	pC->Arm.Links[3].innertia = Vec6SetValues(0.07568, 0, 0, 0.379332, 0, 0.379394);					//link 3 innertial values
	pC->Arm.Links[4].innertia = Vec6SetValues(0.002401, 0, 0,0.002293, 0, 0.002107);					//link 4 innertial values
	pC->Arm.Links[5].innertia = Vec6SetValues(0.002401, 0, 0,0.002293, 0, 0.002107);					//link 5 innertial values
	pC->Arm.Links[6].innertia = Vec6SetValues(0.000001, 0, 0,0.000001, 0, 0.000001);

	pC->Arm.Links[0].mass = 0.750;		//mass of link 0
	pC->Arm.Links[1].mass = 3.545;		//mass of link 1
	pC->Arm.Links[2].mass = 8.78112;	//mass of link 2
	pC->Arm.Links[3].mass = 4.676;		//mass of link 3
	pC->Arm.Links[4].mass = 1.9;		//mass of link 4
	pC->Arm.Links[5].mass = 1.9;		//mass of link 5
	pC->Arm.Links[6].mass = 0.001; 	//mass of link 6 - selected tool + payload
}
static void Control_JtcVariableConf(void)
{
	pC->Jtc.currentFsm = JTC_FSM_Start;
	pC->Jtc.flagInitGetArmModel = true;
	pC->Jtc.flagInitGetFriction = true;
	pC->Jtc.flagInitGetPidParam = true;
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		pC->Jtc.flagInitJointsTab[num] = true;
	
	if(pC->Jtc.flagInitGetFriction == true)
		pC->Jtc.jtcInitStatus |= (1 << 0);
	if(pC->Jtc.flagInitGetPidParam == true)
		pC->Jtc.jtcInitStatus |= (1 << 1);
	if(pC->Jtc.flagInitGetArmModel == true)
		pC->Jtc.jtcInitStatus |= (1 << 2);
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		if(pC->Jtc.flagInitJointsTab[num] == true)
			pC->Jtc.jointsInitStatus |= (1 << num);
		
	pC->Jtc.internalError = false;
	pC->Jtc.externalError = false;
	pC->Jtc.externalJointsError = false;
	pC->Jtc.internalJointsError = false;
	pC->Jtc.internalCanError = false;
	pC->Jtc.internalComError = false;
}
static void Control_RobToolVariableConf(void)
{
	for(int i=0;i<ROBTOOLMAX;i++)
		pC->Jtc.robTools[i].dim = Vec6Zeros();
	
	pC->Jtc.robTools[1].dim = Vec6SetValues(0, 0, 0.14456, 0, 0, 0); //Łącznik ISO z iglicą
	pC->Jtc.robTools[2].dim = Vec6SetValues(-0.020075, 0.063000, 0.251000, 0, 0, 0); //Łącznik ISO z czujnikiem obciążenia
	
	for(int i=0;i<ROBTOOLMAX;i++)
	{
		pC->Jtc.robTools[i].mat = Mat4Ones();
		pC->Jtc.robTools[i].mat = Mat4xMat4(pC->Jtc.robTools[i].mat, HT(pC->Jtc.robTools[i].dim.v[0], pC->Jtc.robTools[i].dim.v[1], pC->Jtc.robTools[i].dim.v[2]));
		pC->Jtc.robTools[i].mat = Mat4xMat4(pC->Jtc.robTools[i].mat, HRZ(pC->Jtc.robTools[i].dim.v[5]));
		pC->Jtc.robTools[i].mat = Mat4xMat4(pC->Jtc.robTools[i].mat, HRY(pC->Jtc.robTools[i].dim.v[4]));
		pC->Jtc.robTools[i].mat = Mat4xMat4(pC->Jtc.robTools[i].mat, HRX(pC->Jtc.robTools[i].dim.v[3]));
		pC->Jtc.robTools[i].matInv = InvMat4(pC->Jtc.robTools[i].mat);
	}
	
	pC->Jtc.robToolNum = 0; //Domyślnie wybrane narzedzie
}
static void Control_JogVariableConf(void)
{
	pC->Jtc.robJog.stepTime = 0.001;
	pC->Jtc.robJog.percentVelPrec = 0.001;
	pC->Jtc.robJog.refSystem = JRS_Joints;
	pC->Jtc.robJog.percentVel = Vec6Zeros();
	for(int i=JRS_Joints;i<JOG_MAXREFSYS;i++)
	{
		pC->Jtc.robJog.maxVel[i] = Vec6SetValues(1, 1, 1, 1, 1, 1);
	}
}
static void Control_TrajClearPointDouble(sTrajPointDouble * p)
{
	for(uint32_t num=0;num<JOINTS_MAX;num++)
	{
		p->pos[num] = 0.;
		p->vel[num] = 0.;
		p->acc[num] = 0.;
	}
}
static void Control_TrajVariableConf(void)
{
	Control_TrajClear();
}
static void Control_VariablesConf(void)
{
	Control_JtcVariableConf();
	Control_TrajVariableConf();
	Joints_SetStartValuesVariables();
	Joints_SetDefaultFriction();
	Joints_SetDefaultPidParam();
	Gripper_SetStartValuesVariables();
	Control_SetDefualtArmModel();
	Control_RobToolVariableConf();
	Control_JogVariableConf();
}
static void Control_ClockConf(void)
{
	//System Clock 480MHz
	uint32_t DIVM1=32, DIVN1=480-1, DIVP1=2-1, DIVQ1=2-1, DIVR1=2-1;
	//FDCAN Clock 85MHz
	uint32_t DIVM2=32, DIVN2=170-1, DIVP2=2-1, DIVQ2=4-1, DIVR2=2-1;
	RCC->CR |= RCC_CR_HSION;
	while(!(RCC->CR & RCC_CR_HSIRDY));
	
	RCC->PLLCKSELR = (DIVM1 << 4) | (DIVM2 << 12);
	RCC->PLL1DIVR = (DIVR1 << 24) | (DIVQ1 << 16) | (DIVP1 << 9) | (DIVN1 << 0);
	RCC->PLL2DIVR = (DIVR2 << 24) | (DIVQ2 << 16) | (DIVP2 << 9) | (DIVN2 << 0);
	RCC->D1CFGR = RCC_D1CFGR_D1CPRE_DIV1 | RCC_D1CFGR_HPRE_DIV2 | RCC_D1CFGR_D1PPRE_DIV2;
	RCC->D2CFGR = RCC_D2CFGR_D2PPRE1_DIV2 | RCC_D2CFGR_D2PPRE2_DIV2;
	RCC->D3CFGR = RCC_D3CFGR_D3PPRE_DIV2;
	
	FLASH->ACR |= FLASH_ACR_WRHIGHFREQ_1 | FLASH_ACR_WRHIGHFREQ_0 | FLASH_ACR_LATENCY_7WS;
	RCC->CR |= RCC_CR_PLL1ON;
	while(!(RCC->CR & RCC_CR_PLL1RDY));
	RCC->CR |= RCC_CR_PLL2ON;
	while(!(RCC->CR & RCC_CR_PLL2RDY));
	RCC->CFGR |= RCC_CFGR_SW_PLL1;
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL1);
}
static void Control_RccConf(void)
{
	RCC->AHB2ENR |= (RCC_AHB2ENR_SRAM1EN | RCC_AHB2ENR_SRAM2EN | RCC_AHB2ENR_SRAM3EN);
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN | RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN;
	RCC->APB1LENR |= RCC_APB1LENR_USART2EN | RCC_APB1LENR_USART3EN | RCC_APB1LENR_TIM6EN | RCC_APB1LENR_TIM7EN | RCC_APB1LENR_TIM13EN | RCC_APB1LENR_TIM14EN | RCC_APB1LENR_TIM12EN | RCC_APB1LENR_TIM5EN;
	RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
	RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
	RCC->D2CCIP1R |= RCC_D2CCIP1R_FDCANSEL_1;
	RCC->AHB3ENR |= RCC_AHB3ENR_AXISRAMEN;
}
static void Control_LedConf(void)
{
	GPIOB->MODER &= ~GPIO_MODER_MODE0;
	GPIOB->MODER 	|= GPIO_MODER_MODE0_0;
	GPIOE->MODER &= ~GPIO_MODER_MODE1;
	GPIOE->MODER 	|= GPIO_MODER_MODE1_0;
	GPIOB->MODER &= ~GPIO_MODER_MODE14;
	GPIOB->MODER 	|= GPIO_MODER_MODE14_0;
}
static void Control_SafetyOutOn(void)
{
//	GPIOD->ODR |= GPIO_ODR_OD3;
	pC->Jtc.emergencyOutput = true;
}
static void Control_SafetyOutOff(void)
{
//	GPIOD->ODR &= ~GPIO_ODR_OD3;
	pC->Jtc.emergencyOutput = false;
}
static bool Control_SafetyInRead(void)
{
	bool in = false;
	if(GPIOD->IDR & GPIO_IDR_ID4)
		in = true;
	return in;
}
static void Control_SafetyConf(void)
{
	//Safety out conf
	GPIOD->MODER &= ~GPIO_MODER_MODE3;
	GPIOD->MODER |= GPIO_MODER_MODE3_0;
	
	//Safety in conf
	GPIOD->MODER &= ~GPIO_MODER_MODE4;
	GPIOD->PUPDR |= GPIO_PUPDR_PUPD4_0;
	
	Control_SafetyOutOff();
}
static void Control_TimerConf(void)
{
	TIM7->CR1 &= ~TIM_CR1_CEN;
	TIM7->CNT = 0;
	TIM7->PSC = 240-1;
	TIM7->ARR = 1000-1;	//Przerwanie co 1 ms
	TIM7->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM7_IRQn);
	TIM7->CR1 |= TIM_CR1_CEN;
}
static void Control_ResetCanDevicesAtBeginig(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
		pC->Joints[num].reqCanReset = true;
	pC->Gripper.reqCanReset = true;
}
void Control_SystemConf(void)
{
	Control_ClockConf();
	SysTick_Config(480000000 / 1000);
	Control_RccConf();
	Control_LedConf();
	Control_VariablesConf();
	MBS_Conf();
	
	#ifdef DEBUG
	Debug_Conf();
	#endif
	
	#ifdef MATLABSIM
	MatlabSim_Conf();
	#endif
	
	TG_Conf();
	Can_Conf();
	RNEA_Conf();
	Kin_Conf();
	Control_SafetyConf();
	IO_Conf();
	Control_ResetCanDevicesAtBeginig();
	Control_TimerConf();
	pC->Jtc.currentFsm = JTC_FSM_Init;
}
void Control_Delay(uint32_t ms)
{
	pC->tick = 0;
	while(pC->tick < ms);
}
// *********************** Joints functions ***************************************
static void Control_JtcSetJointToCurrentMode(uint8_t num)
{
	pC->Joints[num].targetMode = pC->Joints[num].currentMode;
}
static void Control_JtcSetJointToModeTorque(uint8_t num)
{
	pC->Joints[num].targetMode = Joint_M_Torque;
	if(pC->Joints[num].reqIgnore == true)
		return;
	pC->Can.TxMsgs[Can_TxF_ChangeMode].reqSend = true;
}
static void Control_JtcSetJointToCurrentFsm(uint8_t num)
{
	pC->Joints[num].targetFsm = pC->Joints[num].currentFsm;
}
static void Control_JtcSetGripperToCurrentFsm(void)
{
	pC->Gripper.targetFsm = pC->Gripper.currentFsm;
}
static void Control_JtcSetJointToInit(uint8_t num)
{
	pC->Joints[num].targetFsm = Joint_FSM_Init;
	if(pC->Joints[num].reqIgnore == true)
		return;
	pC->Can.TxMsgs[Can_TxF_ChangeFsm].reqSend = true;
}
static void Control_JtcSetJointToReadyToOperate(uint8_t num)
{
	pC->Joints[num].targetFsm = Joint_FSM_ReadyToOperate;
	pC->Joints[num].setTorqueTemp = 0.0;
	if(pC->Joints[num].reqIgnore == true)
		return;
	pC->Can.TxMsgs[Can_TxF_ChangeFsm].reqSend = true;
}
static void Control_JtcSetJointToEnable(uint8_t num)
{
	pC->Joints[num].targetFsm = Joint_FSM_OperationEnable;
	if(pC->Joints[num].reqIgnore == true)
		return;
	pC->Can.TxMsgs[Can_TxF_ChangeFsm].reqSend = true;
}
static void Control_JtcSetGripperToInit(void)
{
	pC->Gripper.targetFsm = Joint_FSM_Init;
	if(pC->Gripper.reqIgnore == true)
		return;
	pC->Can.TxMsgs[Can_TxF_ChangeFsm].reqSend = true;
}
static void Control_JtcSetGripperToReadyToOperate(void)
{
	pC->Gripper.targetFsm = Joint_FSM_ReadyToOperate;
	if(pC->Gripper.reqIgnore == true)
		return;
	pC->Can.TxMsgs[Can_TxF_ChangeFsm].reqSend = true;
}
static void Control_JtcSetGripperToEnable(void)
{
	pC->Gripper.targetFsm = Joint_FSM_OperationEnable;
	if(pC->Gripper.reqIgnore == true)
		return;
	pC->Can.TxMsgs[Can_TxF_ChangeFsm].reqSend = true;
}
static void Control_JtcSetGripperToTargetConf(void)
{
	if(pC->Gripper.reqIgnore == true)
		return;
	pC->Can.TxMsgs[Can_TxF_ChangeMode].reqSend = true;
}
static void Control_JtcReadFrictionFromJoints(void)
{
	pC->Can.TxMsgs[Can_TxF_ReadFriction].reqSend = true;
}
static void Control_CheckLimits(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		// Check torque limits
		if(pC->Joints[num].currentTorque < pC->Joints[num].limitTorqueMin || pC->Joints[num].currentTorque > pC->Joints[num].limitTorqueMax)
			pC->Joints[num].flagSetTorqueOverlimit = true;
		
		if(pC->Joints[num].setTorqueTemp * ((pC->Joints[num].setTorqueTemp < 0) - (0 < pC->Joints[num].setTorqueTemp)) > pC->Joints[num].maxTorqueCom)
			pC->Joints[num].setTorqueTemp = pC->Joints[num].maxTorqueCom * ((pC->Joints[num].setTorqueTemp < 0) - (0 < pC->Joints[num].setTorqueTemp));
		
		// Check pos limits
		if(pC->Joints[num].setPosTemp < pC->Joints[num].limitPosMin || pC->Joints[num].setPosTemp > pC->Joints[num].limitPosMax)
			pC->Joints[num].flagSetPosOverlimit = true;
		
		// Check vel limits
		if(pC->Joints[num].setVelTemp < pC->Joints[num].limitVelMin || pC->Joints[num].setVelTemp > pC->Joints[num].limitVelMax)
			pC->Joints[num].flagSetVelOverlimit = true;
		
		// Check acc limits
		if(pC->Joints[num].setAccTemp < pC->Joints[num].limitAccMin || pC->Joints[num].setAccTemp > pC->Joints[num].limitAccMax)
			pC->Joints[num].flagSetAccOverlimit = true;
		
		// Check pos error limits
		if(pC->Joints[num].pidErrorCurrent < pC->Joints[num].limitPosErrorMin || pC->Joints[num].pidErrorCurrent > pC->Joints[num].limitPosErrorMax)
			pC->Joints[num].flagPosErrorOverlimit = true;
	}
}
static void Control_CheckParkBrakeTimeout(void)
{
	if(pC->Jtc.parkBrakeTimeoutRun == true && IO_ParkBrakeInRead() == true)
	{
		pC->Jtc.parkBrakeTimoeutCnt++;
		if(pC->Jtc.parkBrakeTimoeutCnt > PARKBRAKETIMEOUT)
		{
			pC->Jtc.parkBrakeTimoeutCnt = PARKBRAKETIMEOUT;
			pC->Jtc.internalParkBrakeError = true;
		}
		else
		{
			pC->Jtc.internalParkBrakeError = false;
		}
	}
	else
	{
		pC->Jtc.parkBrakeTimoeutCnt = 0;
	}
}
static void Control_KinNoRealSolutionErrorReact(void)
{

}
static void Control_CheckErrorFlags(void)
{
	pC->Jtc.internalError = false;
	pC->Jtc.externalError = false;
	pC->Jtc.externalWarning = false;
	pC->Jtc.externalJointsWarning = false;
	pC->Jtc.externalJointsError = false;
	pC->Jtc.internalJointsError = false;
	pC->Jtc.internalCanError = false;
	pC->Jtc.internalComError = false;
	pC->Jtc.internalParkBrakeError = false;
	
	// Check Joints internall and external errors
	for(int num=0;num<JOINTS_MAX;num++)
	{
		// Ignorowanie (kasowanie) błędów z danego jointa.
		if(pC->Joints[num].reqIgnore == true)
		{
			pC->Joints[num].currentError = 0x00;
			pC->Joints[num].mcCurrentError = 0x00;
			
			pC->Joints[num].flagSetTorqueOverlimit = false;
			pC->Joints[num].flagFricTableValueOverlimit = false;
			pC->Joints[num].flagSetPosOverlimit = false;
			pC->Joints[num].flagSetVelOverlimit = false;
			pC->Joints[num].flagSetAccOverlimit = false;
			pC->Joints[num].flagPosErrorOverlimit = false;
			
			pC->Joints[num].currentPos = pC->Joints[num].setPosTemp;
		}
		
		// Standardowa procedura obsługi błędów z danego jointa
		pC->Joints[num].flagCanError = false;
		pC->Joints[num].flagJtcError = false;
		
		if(pC->Joints[num].currentError != 0x00)										pC->Joints[num].flagCanError = true;
		if(pC->Joints[num].mcCurrentError != 0x00)									pC->Joints[num].flagCanError = true;
		
		if(pC->Joints[num].flagSetTorqueOverlimit == true)					pC->Joints[num].flagJtcError = true;
		if(pC->Joints[num].flagFricTableValueOverlimit == true)			pC->Joints[num].flagJtcError = true;
		if(pC->Joints[num].flagSetPosOverlimit == true)							pC->Joints[num].flagJtcError = true;
		if(pC->Joints[num].flagSetVelOverlimit == true)							pC->Joints[num].flagJtcError = true;
		if(pC->Joints[num].flagSetAccOverlimit == true)							pC->Joints[num].flagJtcError = true;
		if(pC->Joints[num].flagPosErrorOverlimit == true)						pC->Joints[num].flagJtcError = true;
		
		if(pC->Joints[num].flagCanError == true)										pC->Jtc.externalJointsError = true;
		if(pC->Joints[num].flagJtcError == true)										pC->Jtc.internalJointsError = true;
	}
	
	if(pC->Gripper.reqIgnore == true)
	{
		pC->Gripper.currentError = 0x00;
		pC->Gripper.flagCanError = false;
		pC->Gripper.flagJtcError = false;
	}
	if(pC->Gripper.currentError != 0x00)													pC->Gripper.flagCanError = true;
	if(pC->Gripper.flagCanError == true)													pC->Jtc.externalJointsError = true;
	if(pC->Gripper.flagJtcError == true)													pC->Jtc.internalJointsError = true;
	
	// Check Park Brake Timeout
	Control_CheckParkBrakeTimeout();
	
	// Reakcja na bląd z kinematyki odwrotnej dotyczący braku rzeczywistych rozwiazan
	Control_KinNoRealSolutionErrorReact();
	
	// Check CAN comunication errors
	if(pC->Can.statusId == Can_SId_Error)
		pC->Jtc.internalCanError = true;
	
	// Check hardware emergency line
//	pC->Jtc.emergencyInput = Control_SafetyInRead();
	
	//Check global internall error
	if(pC->Jtc.internalJointsError == true || pC->Jtc.internalCanError == true || pC->Jtc.internalComError == true || pC->Jtc.internalParkBrakeError == true || pC->Jtc.internalKinNoRealSoution == true)
	{
		pC->Jtc.internalError = true;
		Control_SafetyOutOn();
	}
	else
	{
		Control_SafetyOutOff();
	}
	//Check global external error
	if(pC->Jtc.externalJointsError == true || pC->Jtc.emergencyInput == true)
		pC->Jtc.externalError = true;
	
	//Check global external warning
	if(pC->Jtc.externalJointsWarning == true)
		pC->Jtc.externalWarning = true;
	
	// Przygotowywanie flag bledów JTC do wyslania
	pC->Jtc.errors = 0x0000;
	pC->Jtc.errors |= pC->Jtc.emergencyInput << 0; 						// bit 0
	pC->Jtc.errors |= pC->Jtc.emergencyOutput << 1; 					// bit 1
	pC->Jtc.errors |= pC->Jtc.internalError << 2; 						// bit 2
	pC->Jtc.errors |= pC->Jtc.externalError << 3; 						// bit 3
	pC->Jtc.errors |= pC->Jtc.internalJointsError << 4; 			// bit 4
	pC->Jtc.errors |= pC->Jtc.internalCanError << 5; 					// bit 5
	pC->Jtc.errors |= pC->Jtc.internalComError << 6; 					// bit 6
	pC->Jtc.errors |= pC->Jtc.externalJointsError << 7; 			// bit 7
	pC->Jtc.errors |= pC->Jtc.internalParkBrakeError << 8; 		// bit 8
	pC->Jtc.errors |= pC->Jtc.internalKinNoRealSoution << 9; 	// bit 9
	
	pC->Jtc.occuredErrors |= pC->Jtc.errors;
	
	// Przygotowywanie flag bledów wewnetrznych jointow do wyslania
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].internallErrors = 0x0000;
		pC->Joints[num].internallErrors |= pC->Joints[num].flagSetPosOverlimit << 0;							// bit 0
		pC->Joints[num].internallErrors |= pC->Joints[num].flagSetVelOverlimit << 1;							// bit 1
		pC->Joints[num].internallErrors |= pC->Joints[num].flagSetAccOverlimit << 2;							// bit 2
		pC->Joints[num].internallErrors |= pC->Joints[num].flagSetTorqueOverlimit << 3;						// bit 3
		pC->Joints[num].internallErrors |= pC->Joints[num].flagPosErrorOverlimit << 4;						// bit 4
		pC->Joints[num].internallErrors |= pC->Joints[num].flagFricTableValueOverlimit << 5;			// bit 5
		
		pC->Joints[num].internallOccuredErrors |= pC->Joints[num].internallErrors;
	}
}
static void Control_SetNewTorqueValues(void)
{
	if(pC->Jtc.externalError == false && pC->Jtc.internalError == false)
	{
		for(uint8_t num=0;num<JOINTS_MAX;num++)
			pC->Joints[num].setTorque = pC->Joints[num].setTorqueTemp;
	}
	else
	{
		for(uint8_t num=0;num<JOINTS_MAX;num++)
		{
			pC->Joints[num].setTorque = 0.0;
			Control_JtcSetJointToReadyToOperate(num);
		}
	}
}
static void Control_SendCommandClearErrorsToJoints(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		if(pC->Joints[num].reqCanClearErrors == true)
		{
			Control_JtcSetJointToInit(num);
			pC->Joints[num].reqCanClearErrors = false;
		}
	}
	if(pC->Gripper.reqCanClearErrors == true)
	{
		Control_JtcSetGripperToInit();
		pC->Gripper.reqCanClearErrors = false;
	}
}
static void Control_SendCommandResetDevice(void)
{
	//sprawdzenie czy wszystkie urzadzenia wymagaja resetowania
	bool flag = true;
	for(int num=0;num<JOINTS_MAX;num++)
		if(pC->Joints[num].reqCanReset == false)
			flag = false;
	if(pC->Gripper.reqCanReset == false)
			flag = false;
	
	if(flag == true && pC->Can.TxMsgs[Can_TxF_ResetAllDevices].timeoutCnt > CAN_RESETTIMEOUT)
	{
		for(int num=0;num<JOINTS_MAX;num++)
		{
			pC->Joints[num].reqCanReset = false;
			Joints_SetResetValuesVariables(num);
		}
		pC->Gripper.reqCanReset = false;
		Gripper_SetResetValuesVariables();
		
		pC->Can.TxMsgs[Can_TxF_ResetAllDevices].reqSend = true;
		return;
	}
	
	//indywidualne resetowanie urzadzen
	if(pC->Joints[0].reqCanReset == true && pC->Can.TxMsgs[Can_TxF_ResetJoint0].timeoutCnt > CAN_RESETTIMEOUT)
	{
		pC->Joints[0].reqCanReset = false;
		Joints_SetResetValuesVariables(0);
		pC->Can.TxMsgs[Can_TxF_ResetJoint0].reqSend = true;
	}
	if(pC->Joints[1].reqCanReset == true && pC->Can.TxMsgs[Can_TxF_ResetJoint1].timeoutCnt > CAN_RESETTIMEOUT)
	{
		pC->Joints[1].reqCanReset = false;
		Joints_SetResetValuesVariables(1);
		pC->Can.TxMsgs[Can_TxF_ResetJoint1].reqSend = true;
	}
	if(pC->Joints[2].reqCanReset == true && pC->Can.TxMsgs[Can_TxF_ResetJoint2].timeoutCnt > CAN_RESETTIMEOUT)
	{
		pC->Joints[2].reqCanReset = false;
		Joints_SetResetValuesVariables(2);
		pC->Can.TxMsgs[Can_TxF_ResetJoint2].reqSend = true;
	}
	if(pC->Joints[3].reqCanReset == true && pC->Can.TxMsgs[Can_TxF_ResetJoint3].timeoutCnt > CAN_RESETTIMEOUT)
	{
		pC->Joints[3].reqCanReset = false;
		Joints_SetResetValuesVariables(3);
		pC->Can.TxMsgs[Can_TxF_ResetJoint3].reqSend = true;
	}
	if(pC->Joints[4].reqCanReset == true && pC->Can.TxMsgs[Can_TxF_ResetJoint4].timeoutCnt > CAN_RESETTIMEOUT)
	{
		pC->Joints[4].reqCanReset = false;
		Joints_SetResetValuesVariables(4);
		pC->Can.TxMsgs[Can_TxF_ResetJoint4].reqSend = true;
	}
	if(pC->Joints[5].reqCanReset == true && pC->Can.TxMsgs[Can_TxF_ResetJoint5].timeoutCnt > CAN_RESETTIMEOUT)
	{
		pC->Joints[5].reqCanReset = false;
		Joints_SetResetValuesVariables(5);
		pC->Can.TxMsgs[Can_TxF_ResetJoint5].reqSend = true;
	}
	if(pC->Gripper.reqCanReset == true && pC->Can.TxMsgs[Can_TxF_ResetGripper].timeoutCnt > CAN_RESETTIMEOUT)
	{
		pC->Gripper.reqCanReset = false;
		Gripper_SetResetValuesVariables();
		pC->Can.TxMsgs[Can_TxF_ResetGripper].reqSend = true;
	}
}
static void Control_SendDataToJoints(void)
{
	Can_SendDataToJoints();
}
static void Control_SetVariableForIgnoreDevices(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		if(pC->Joints[num].reqIgnore == true)
		{
			pC->Joints[num].pidTorque = 0.0;
			pC->Joints[num].idTorque = 0.0;
			pC->Joints[num].fricTorque = 0.0;
			pC->Joints[num].setTorqueTemp = 0.0;
		}
	}
}
// ***************** Trajectory functions ************************************
void Control_TrajClear(void)
{
	if(Traj.currentTES == TES_Null)
		return;
	
	Traj.comStatus = TCS_Null;
	Traj.targetTES = TES_Null;
	Traj.currentTES = TES_Null;
	Traj.numTraj = 0;
	Traj.maxPoints = 0;
	Traj.stepTime = 0;
	Traj.numRecPoints = 0;
	
	Traj.maxInterPoints = 0;
	Traj.numInterPoint = 0;
	
	Control_TrajClearPointDouble(&Traj.startPoint);
	Control_TrajClearPointDouble(&Traj.interpolatePoint);
	Control_TrajClearPointDouble(&Traj.endPoint);
}
static void Control_TrajInterpolate(void)
{
	if(Traj.stepTime == 0)
		return;
	if((Traj.numInterPoint + 1) >= Traj.maxInterPoints)
		return;

	uint16_t n = Traj.numInterPoint / Traj.stepTime; // numer punktu z trajektorii (zakres od 0 do Traj.maxPoints-1)
	uint16_t m = Traj.numInterPoint % Traj.stepTime; // numer kroku w interpolaji pomiedzy punktem poczatkowym a koncowym (zakres od 0 do Traj.stepTime-1)
	
	sTrajPoint start = Traj.points[n];
	sTrajPoint end = Traj.points[n + 1];
	
	
	for(int num=0;num<JOINTS_MAX;num++)
	{
		Traj.startPoint.pos[num] = (double)start.pos[num] * pC->Joints[num].limitPosMax / MAXINT16;
		Traj.startPoint.vel[num] = (double)start.vel[num] * pC->Joints[num].limitVelMax / MAXINT16;
		Traj.startPoint.acc[num] = (double)start.acc[num] * pC->Joints[num].limitAccMax / MAXINT16;
		
		Traj.endPoint.pos[num] = (double)end.pos[num] * pC->Joints[num].limitPosMax / MAXINT16;
		Traj.endPoint.vel[num] = (double)end.vel[num] * pC->Joints[num].limitVelMax / MAXINT16;
		Traj.endPoint.acc[num] = (double)end.acc[num] * pC->Joints[num].limitAccMax / MAXINT16;
	}
	
	for(int num=0;num<JOINTS_MAX;num++)
	{
		double dPos = Traj.endPoint.pos[num] - Traj.startPoint.pos[num];
		double offsetPos = (double)m * (dPos / (double)Traj.stepTime);
		Traj.interpolatePoint.pos[num] = Traj.startPoint.pos[num] + offsetPos;
		
		double dVel = Traj.endPoint.vel[num] - Traj.startPoint.vel[num];
		double offsetVel = (double)m * (dVel / (double)Traj.stepTime);
		Traj.interpolatePoint.vel[num] = Traj.startPoint.vel[num] + offsetVel;
		
		double dAcc = Traj.endPoint.acc[num] - Traj.startPoint.acc[num];
		double offsetAcc = (double)m * (dAcc / (double)Traj.stepTime);
		Traj.interpolatePoint.acc[num] = Traj.startPoint.acc[num] + offsetAcc;
	}
}
static void Control_TrajCheckState(void)
{
	if(Traj.currentTES == TES_Null && Traj.targetTES == TES_Stop)
		Traj.currentTES = TES_TransNullToStop; 				// Przejscie przez stan posredni
	else if(Traj.currentTES == TES_Stop && Traj.targetTES == TES_Execute)
		Traj.currentTES = TES_Execute; 								// Przejscie natychmiastowe
	else if(Traj.currentTES == TES_Pause && Traj.targetTES == TES_Stop)
		Traj.currentTES = TES_Stop; 									// Przejscie natychmiastowe
	else if(Traj.currentTES == TES_Pause && Traj.targetTES == TES_Execute)
		Traj.currentTES = TES_Execute; 								// Przejscie natychmiastowe
	else if(Traj.currentTES == TES_Execute && Traj.targetTES == TES_Pause)
		Traj.currentTES = TES_Pause; 									// Przejscie natychmiastowe
	else if(Traj.currentTES == TES_Execute && Traj.targetTES == TES_Stop)
		Traj.currentTES = TES_Stop; 									// Przejscie natychmiastowe
	else if(Traj.currentTES == TES_Execute && Traj.targetTES == TES_Finish)
		Traj.currentTES = TES_Finish; 								// Przejscie natychmiastowe
	else if(Traj.currentTES == TES_Finish && Traj.targetTES == TES_Stop)
		Traj.currentTES = TES_Stop; 									// Przejscie natychmiastowe
	
	
	// Check JTC status: HoldPos
	if(Traj.currentTES == TES_Null || Traj.currentTES == TES_Stop || Traj.currentTES == TES_Pause || Traj.currentTES == TES_Finish ||  Traj.currentTES == TES_TransNullToStop)
		pC->Jtc.holdposModeReq = true;
	else
		pC->Jtc.holdposModeReq = false;
	// Check JTC status: Operate
	if(Traj.currentTES == TES_Execute)
		pC->Jtc.operateModeReq = true;
	else
		pC->Jtc.operateModeReq = false;
}
static void Control_TrajStop(void)
{
	Traj.maxPoints = Traj.numRecPoints;
	Traj.maxInterPoints = (Traj.maxPoints-1) * Traj.stepTime;
	Traj.numInterPoint = 0;
	
	Control_TrajClearPointDouble(&Traj.startPoint);
	Control_TrajClearPointDouble(&Traj.interpolatePoint);
	Control_TrajClearPointDouble(&Traj.endPoint);
}
static void Control_TrajTransNullToStop(void)
{
	if(Traj.comStatus == TCS_WasRead)
	{
		Traj.currentTES = TES_Stop;
	}
	else
	{
		Traj.targetTES = TES_Null;
		Traj.currentTES = TES_Null;
	}
}
// ***************** Joint Trajectory Controller functions *********************
static void Control_JtcPrepareSetedValuesForInit(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].idSetPos = pC->Joints[num].currentPos;		// aktualna pozycja katowa napedu dla dynamiki
		pC->Joints[num].idSetVel = 0.0;
		pC->Joints[num].idSetAcc = 0.0;
	}
}
static void Control_JtcPrepareSetedValuesForTeaching(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].idSetPos = pC->Joints[num].currentPos;		// aktualna pozycja katowa napedu dla dynamiki
		pC->Joints[num].idSetVel = 0.0;
		pC->Joints[num].idSetAcc = 0.0;
	}
}
static void Control_JtcPrepareSetedValuesForTeachingConstTorque(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].idSetPos = pC->Joints[num].currentPos;		// aktualna pozycja katowa napedu dla dynamiki
		pC->Joints[num].idSetVel = 0.0;
		pC->Joints[num].idSetAcc = 0.0;
	}
}
static void Control_JtcPrepareSetedValuesForHoldPos(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].idSetPos = pC->Joints[num].currentPos;		// aktualna pozycja katowa napedu dla dynamiki
		pC->Joints[num].idSetVel = 0.0;
		pC->Joints[num].idSetAcc = 0.0;
	}
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].setPosTemp = pC->Joints[num].setPosTemp; 	//utrzymywanie zadanej pozycji
		pC->Joints[num].setVelTemp = 0.0;
		pC->Joints[num].setAccTemp = 0.0;
	}
}
static void Control_JtcJogActJRSJoints(void)
{
	if(pC->Jtc.robJog.active[JRS_Joints] == false)
	{
		for(int i=0;i<JOG_MAXREFSYS;i++)
			pC->Jtc.robJog.active[i] = false;
		pC->Jtc.robJog.active[JRS_Joints] = true;
	}
	pC->Jtc.robJog.kinStatus = JKS_Idle;
	
	for(int num=0;num<JOINTS_MAX;num++)
	{
		if(fabs(pC->Jtc.robJog.percentVel.v[num]) < pC->Jtc.robJog.percentVelPrec)
			continue;
		
		double tempPos = pC->Joints[num].setPosTemp + (pC->Jtc.robJog.percentVel.v[num] / 100.0) * pC->Jtc.robJog.maxVel[JRS_Joints].v[num] * pC->Jtc.robJog.stepTime;
		if(pC->Jtc.robJog.percentVel.v[num] > pC->Jtc.robJog.percentVelPrec && tempPos > pC->Joints[num].limitPosMax)
			continue;
		if(pC->Jtc.robJog.percentVel.v[num] < -pC->Jtc.robJog.percentVelPrec && tempPos < pC->Joints[num].limitPosMin)
			continue;
		pC->Joints[num].setPosTemp = tempPos;
	}
}
static void Control_JtcJogActJRSBase(void)
{
	if(pC->Jtc.robJog.active[JRS_Base] == false)
	{
		for(int i=0;i<JOG_MAXREFSYS;i++)
			pC->Jtc.robJog.active[i] = false;
		pC->Jtc.robJog.active[JRS_Base] = true;
		pC->Jtc.robJog.targetPos = pC->Jtc.robPos;
		for(int num=0;num<JOINTS_MAX;num++)
			pC->Joints[num].setPosTemp = pC->Joints[num].currentPos;
	}
	
	sVector6 delta = Vec6Zeros();
	bool flag = false;
	for(int num=0;num<6;num++)
	{
		if(fabs(pC->Jtc.robJog.percentVel.v[num]) < pC->Jtc.robJog.percentVelPrec)
			continue;
		delta.v[num] = (pC->Jtc.robJog.percentVel.v[num] / 100.0) * pC->Jtc.robJog.maxVel[JRS_Base].v[num] * pC->Jtc.robJog.stepTime;
		flag = true;
	}
	
	if(flag == false && pC->Jtc.robJog.kinStatus == JKS_Idle)
		return;

	sMatrix4 deltaMat = Mat4Ones();
	deltaMat = Mat4xMat4(deltaMat, HRZ(delta.v[5]));
	deltaMat = Mat4xMat4(deltaMat, HRY(delta.v[4]));
	deltaMat = Mat4xMat4(deltaMat, HRX(delta.v[3]));
	pC->Jtc.robJog.targetPos.mat = Mat3ToMat4(Mat3xMat3(Mat4ToMat3(deltaMat), Mat4ToMat3(pC->Jtc.robJog.targetPos.mat)), pC->Jtc.robJog.targetPos.mat);

	for(int i=0;i<3;i++)
		pC->Jtc.robJog.targetPos.mat.v[i][3] += delta.v[i];

	if(pC->Jtc.robJog.kinStatus == JKS_Idle)
		pC->Jtc.robJog.kinStatus = JKS_InputIsReady;
	
	if(pC->Jtc.robJog.kinStatus == JKS_OutputIsReady)
	{
		pC->Jtc.robJog.targetPos = Kin_FindNearestSolution(pC->Jtc.robJog.targetPos);
		for(int num=0;num<JOINTS_MAX;num++)
			pC->Joints[num].setPosTemp = pC->Jtc.robJog.targetPos.qSol.v[num];
		pC->Jtc.robJog.kinStatus = JKS_Idle;
	}
}
static void Control_JtcJogActJRSTool(void)
{
	if(pC->Jtc.robJog.active[JRS_Tool] == false)
	{
		for(int i=0;i<JOG_MAXREFSYS;i++)
			pC->Jtc.robJog.active[i] = false;
		pC->Jtc.robJog.active[JRS_Tool] = true;
		pC->Jtc.robJog.targetPos = pC->Jtc.robPos;
		for(int num=0;num<JOINTS_MAX;num++)
			pC->Joints[num].setPosTemp = pC->Joints[num].currentPos;
	}
	
	sVector6 delta = Vec6Zeros();
	bool flag = false;
	for(int num=0;num<6;num++)
	{
		if(fabs(pC->Jtc.robJog.percentVel.v[num]) < pC->Jtc.robJog.percentVelPrec)
			continue;
		delta.v[num] = (pC->Jtc.robJog.percentVel.v[num] / 100.0) * pC->Jtc.robJog.maxVel[JRS_Tool].v[num] * pC->Jtc.robJog.stepTime;
		flag = true;
	}
	
	if(flag == false && pC->Jtc.robJog.kinStatus == JKS_Idle)
		return;
	
	sMatrix4 deltaMat = Mat4Ones();
	deltaMat = Mat4xMat4(deltaMat, HT(delta.v[0], delta.v[1], delta.v[2]));
	deltaMat = Mat4xMat4(deltaMat, HRZ(delta.v[5]));
	deltaMat = Mat4xMat4(deltaMat, HRY(delta.v[4]));
	deltaMat = Mat4xMat4(deltaMat, HRX(delta.v[3]));
	pC->Jtc.robJog.targetPos.mat = Mat4xMat4(pC->Jtc.robJog.targetPos.mat, deltaMat);
	
	if(pC->Jtc.robJog.kinStatus == JKS_Idle)
		pC->Jtc.robJog.kinStatus = JKS_InputIsReady;
	
	if(pC->Jtc.robJog.kinStatus == JKS_OutputIsReady)
	{
		pC->Jtc.robJog.targetPos = Kin_FindNearestSolution(pC->Jtc.robJog.targetPos);
		for(int num=0;num<JOINTS_MAX;num++)
			pC->Joints[num].setPosTemp = pC->Jtc.robJog.targetPos.qSol.v[num];
		pC->Jtc.robJog.kinStatus = JKS_Idle;
	}
}
static void Control_JtcJogAct(void)
{
	if(pC->Jtc.robJog.refSystem == JRS_Joints)
	{
		Control_JtcJogActJRSJoints();
	}
	else if(pC->Jtc.robJog.refSystem == JRS_Base)
	{
		Control_JtcJogActJRSBase();
	}
	else if(pC->Jtc.robJog.refSystem == JRS_Tool)
	{
		Control_JtcJogActJRSTool();
	}
}
void Control_JtcJogKinCalc(void)
{
	if(pC->Jtc.robJog.kinStatus == JKS_InputIsReady)
	{
		pC->Jtc.robJog.targetPos = Kin_IKCalcFromRotMat(pC->Jtc.robJog.targetPos);
		pC->Jtc.robJog.kinStatus = JKS_OutputIsReady;
	}
}
void Control_JtcReqChangeTool(uint16_t num)
{
	if(num > ROBTOOLMAX || num == pC->Jtc.targetRobToolNum)
		return;
	pC->Jtc.targetRobToolNum = num;
	pC->Jtc.reqChangeTool = true;
}
static void Control_JtcChangeTool(void)
{
	if(pC->Jtc.reqChangeTool == false)
	{
		return;
	}
	else if(pC->Jtc.targetRobToolNum == pC->Jtc.robToolNum)
	{
		pC->Jtc.reqChangeTool = false;
		return;
	}
	else if(pC->Jtc.robJog.kinStatus != JKS_Idle)
	{
		return;
	}
	
	pC->Jtc.robToolNum = pC->Jtc.targetRobToolNum;
	Kin_RobPosAct();
	pC->Jtc.reqChangeTool = false;
	for(int i=0;i<JOG_MAXREFSYS;i++)
		pC->Jtc.robJog.active[i] = false;
}
static void Control_JtcPrepareSetedValuesForOperate(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].idSetPos = pC->Joints[num].currentPos;					// aktualna pozycja katowa napedu dla dynamiki
		pC->Joints[num].idSetVel = Traj.interpolatePoint.vel[num];			// przewidywana zadana predkosc katowa tymczasowa
		pC->Joints[num].idSetAcc = Traj.interpolatePoint.acc[num];			// przewidywane zadane przyspieszenie katowe tymczasowa
	}
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].setPosTemp = Traj.interpolatePoint.pos[num];
		pC->Joints[num].setVelTemp = Traj.interpolatePoint.vel[num];
		pC->Joints[num].setAccTemp = Traj.interpolatePoint.acc[num];
	}
}
void Control_ClearInternallErrorsInJtc(void)
{
	//Clear Jtc Errors
	pC->Jtc.emergencyInput = false;
	pC->Jtc.emergencyOutput = false;
	pC->Jtc.internalError = false;
	pC->Jtc.externalError = false;
	pC->Jtc.externalWarning = false;
	pC->Jtc.internalJointsError = false;
	pC->Jtc.internalCanError = false;
	pC->Jtc.internalComError = false;
	pC->Jtc.externalJointsError = false;
	pC->Jtc.externalJointsWarning = false;
	pC->Jtc.parkBrakeTimoeutCnt = 0;
	pC->Jtc.internalParkBrakeError = false;
	pC->Jtc.internalKinNoRealSoution = false;
	pC->Jtc.errors = 0x00;
	pC->Jtc.occuredErrors = 0x00;
	
	//Clear internall device errors
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].flagCanError = false;
		pC->Joints[num].flagJtcError = false;
		pC->Joints[num].flagSetTorqueOverlimit = false;
		pC->Joints[num].flagFricTableValueOverlimit = false;
		pC->Joints[num].flagSetPosOverlimit = false;
		pC->Joints[num].flagSetVelOverlimit = false;
		pC->Joints[num].flagSetAccOverlimit = false;
		pC->Joints[num].flagPosErrorOverlimit = false;
		pC->Joints[num].mcCurrentError = 0x00;
		pC->Joints[num].mcOccuredError = 0x00;
		pC->Joints[num].currentError = 0x00;
		pC->Joints[num].currentWarning = 0x00;
		pC->Joints[num].internallErrors = 0x00;
		pC->Joints[num].internallOccuredErrors = 0x00;
	}
	pC->Gripper.flagCanError = false;
	pC->Gripper.flagJtcError = false;
	pC->Gripper.currentError = 0x00;
	pC->Gripper.currentWarning = 0x00;
	pC->Gripper.internallErrors = 0x00;
	pC->Gripper.internallOccuredErrors = 0x00;
	
	//Clear Can Errors
	for(int num=0;num<CAN_RXBUF_MAX;num++)
	{
		pC->Can.RxMsgs[num].flagTimeout = false;
		pC->Can.RxMsgs[num].timeoutCnt = 0;
	}
	for(int num=0;num<CAN_TXBUF_MAX;num++)
	{
		pC->Can.TxMsgs[num].flagTimeout = false;
		pC->Can.TxMsgs[num].timeoutCnt = 0;
	}
	pC->Can.statusId = Can_SId_NoError;
	pC->Can.statusFlags = 0x00;
	pC->Can.statusOccurredFlags = 0x00;
	
	// Check new errors
	Control_CheckErrorFlags();
}
void Control_ClearExternallErrorsViaCan(uint8_t byte)
{
	if(((byte >> Can_DN_Joint0) & 0x01) == 0x01) 		pC->Joints[Can_DN_Joint0].reqCanClearErrors = true;
	if(((byte >> Can_DN_Joint1) & 0x01) == 0x01) 		pC->Joints[Can_DN_Joint1].reqCanClearErrors = true;
	if(((byte >> Can_DN_Joint2) & 0x01) == 0x01) 		pC->Joints[Can_DN_Joint2].reqCanClearErrors = true;
	if(((byte >> Can_DN_Joint3) & 0x01) == 0x01) 		pC->Joints[Can_DN_Joint3].reqCanClearErrors = true;
	if(((byte >> Can_DN_Joint4) & 0x01) == 0x01) 		pC->Joints[Can_DN_Joint4].reqCanClearErrors = true;
	if(((byte >> Can_DN_Joint5) & 0x01) == 0x01) 		pC->Joints[Can_DN_Joint5].reqCanClearErrors = true;
	if(((byte >> Can_DN_Gripper) & 0x01) == 0x01) 	pC->Gripper.reqCanClearErrors = true;
}
void Control_ResetDevicesViaCan(uint8_t data)
{
	for(int num=0;num<JOINTS_MAX;num++)
		Joints_SetResetValuesVariables(num);
	Gripper_SetResetValuesVariables();
	
	pC->Joints[Can_DN_Joint0].reqCanReset = ((data >> Can_DN_Joint0) & 0x01);
	pC->Joints[Can_DN_Joint1].reqCanReset = ((data >> Can_DN_Joint1) & 0x01);
	pC->Joints[Can_DN_Joint2].reqCanReset = ((data >> Can_DN_Joint2) & 0x01);
	pC->Joints[Can_DN_Joint3].reqCanReset = ((data >> Can_DN_Joint3) & 0x01);
	pC->Joints[Can_DN_Joint4].reqCanReset = ((data >> Can_DN_Joint4) & 0x01);
	pC->Joints[Can_DN_Joint5].reqCanReset = ((data >> Can_DN_Joint5) & 0x01);
	pC->Gripper.reqCanReset = ((data >> Can_DN_Gripper) & 0x01);
}
static void Control_JtcCheckStateError(void)
{
	// Error check
	if(pC->Jtc.internalError == true || pC->Jtc.externalError == true)
		pC->Jtc.errorModeReq = true;
	else
		pC->Jtc.errorModeReq = false;
}
static void Control_JtcCheckStateInit(void)
{
	bool flag = true;
	
	// **************************************************************************************************************************************************
	// Sprawdzanie w jakiej fazie inicjalizacji jest JTC. Możliwe fazy: JTC_IS_PreInit, JTC_IS_Depark, JTC_IS_RemoveBrake, JTC_IS_Finish
	pC->Jtc.initStage = JTC_IS_PreInit; //Startowe ustalenie wartości
	
	// ----------- Faza inicjalizacji JTC_IS_PreInit ----------------------------------------------------------------------------------------------------
	if(pC->Jtc.initStage == JTC_IS_PreInit)
	{
		// Ignorowanie grippera podczas wstępnej inicjalizacji (wpisywane wartości tak jakby gripper przeszedł poprawnie fazę wstępnej inicjalizacji)
		if(pC->Gripper.reqIgnore == true)
		{
			pC->Gripper.flagFirstPosRead = true;
			pC->Gripper.flagConfirmChangeConf = true;
			pC->Gripper.currentFsm = Joint_FSM_ReadyToOperate;
		}
		// Ignorowanie danego jointa podczas wstępnej inicjalizacji (wpisywane wartości tak jakby joint przeszedł poprawnie fazę wstępnej inicjalizacji)
		for(int num=0;num<JOINTS_MAX;num++)
		{
			if(pC->Joints[num].reqIgnore == true)
			{
				pC->Joints[num].flagFirstPosRead = true;
				pC->Joints[num].flagConfirmChangeConf = true;
				pC->Joints[num].flagFrictionReadFromCan = true;
				pC->Joints[num].currentFsm = Joint_FSM_ReadyToOperate;
				pC->Joints[num].currentMode = Joint_M_Torque;
			}
		}
		// Sprawdzanie parametrów grippera i jointów
		flag = true;
		if(pC->Gripper.flagConfirmChangeConf != true)							flag = false;
		if(pC->Gripper.currentFsm == Joint_FSM_Init)							flag = false;
		if(pC->Gripper.currentFsm == Joint_FSM_Start)							flag = false;
		for(uint8_t num=0;num<JOINTS_MAX;num++)
		{
			if(pC->Joints[num].flagConfirmChangeConf != true)				flag = false;
			if(pC->Joints[num].flagFrictionReadFromCan != true)			flag = false;
			if(pC->Joints[num].currentFsm == Joint_FSM_Init)				flag = false;
			if(pC->Joints[num].currentFsm == Joint_FSM_Start)				flag = false;
		}
		// Można przejść do następnej fazy inicjalizacji Jtc
		if(flag == true)
			pC->Jtc.initStage = JTC_IS_Depark;
	}
	
	// ----------- Faza inicjalizacji JTC_IS_Depark ----------------------------------------------------------------------------------------------------
	if(pC->Jtc.initStage == JTC_IS_Depark)
	{
		// Ignorowanie grippera - w tej fazie inicjalizacji nie ma znaczenia
		// Ignorowanie danego jointa podczas inicjalizacji JTC_IS_Depark (wpisywane wartości tak jakby joint przeszedł poprawnie fazę inicjalizacji JTC_IS_Depark)
		for(int num=0;num<JOINTS_MAX;num++)
			if(pC->Joints[num].reqIgnore == true)
				pC->Joints[num].flagDeparkPosAchieved = true;
		
		// Sprawdzanie parametrów jointów i odblokowanie hamulca
		flag = true;
		for(int num=0;num<JOINTS_MAX;num++)
			if(pC->Joints[num].flagDeparkPosAchieved == false)
				flag = false; //Joint nie osiągnął zadanej pozycji odsunięcia się od sworznia hamulca
		
		// Można przejść do następnej fazy inicjalizacji Jtc
		if(flag == true)
		{
			if(pC->Jtc.currentFsm == JTC_FSM_Init || pC->Jtc.currentFsm == JTC_FSM_Error)
				pC->Jtc.initStage = JTC_IS_RemoveBrake;
			else
				pC->Jtc.initStage = JTC_IS_Finish;
		}
	}
	
	// ----------- Faza inicjalizacji JTC_IS_RemoveBrake ----------------------------------------------------------------------------------------------------
	if(pC->Jtc.initStage == JTC_IS_RemoveBrake)
	{
		if(IO_ParkBrakeInRead() == false)
			pC->Jtc.initStage = JTC_IS_Finish; //Hamulec został odblokowany
	}
	
	// ----------- Faza inicjalizacji JTC_IS_Finish ----------------------------------------------------------------------------------------------------
	if(pC->Jtc.initStage == JTC_IS_Finish)
	{
		IO_ParkBrakeLock();
	}

	// Check Gripper init status flags
	if(pC->Gripper.flagConfirmChangeConf == true && (pC->Gripper.currentFsm == Joint_FSM_ReadyToOperate || pC->Gripper.currentFsm == Joint_FSM_OperationEnable))
		pC->Jtc.jointsInitStatus &= ~(1 << Can_DN_Gripper);
	else if(pC->Gripper.flagConfirmChangeConf != true || pC->Gripper.currentFsm == Joint_FSM_Init || pC->Gripper.currentFsm == Joint_FSM_Start)
		pC->Jtc.jointsInitStatus |= (1 << Can_DN_Gripper);
	
	// Check Joints init status flags
	for(uint8_t num=0;num<JOINTS_MAX;num++)
	{
		if(pC->Joints[num].flagDeparkPosAchieved == true && pC->Joints[num].flagConfirmChangeConf == true && (pC->Joints[num].currentFsm == Joint_FSM_ReadyToOperate || pC->Joints[num].currentFsm == Joint_FSM_OperationEnable) && pC->Joints[num].flagFrictionReadFromCan == true)
			pC->Jtc.jointsInitStatus &= ~(1 << num);
		else if(pC->Joints[num].flagDeparkPosAchieved != true || pC->Joints[num].flagConfirmChangeConf != true || pC->Joints[num].currentFsm == Joint_FSM_Init || pC->Joints[num].currentFsm == Joint_FSM_Start || pC->Joints[num].flagFrictionReadFromCan == false)
			pC->Jtc.jointsInitStatus |= (1 << num);
	}
	
	// Check JTC init status flags
	if(pC->Jtc.flagInitGetFriction == false)
		pC->Jtc.jtcInitStatus &= ~(1 << 0);
	if(pC->Jtc.flagInitGetPidParam == false)
		pC->Jtc.jtcInitStatus &= ~(1 << 1);
	if(pC->Jtc.flagInitGetArmModel == false)
		pC->Jtc.jtcInitStatus &= ~(1 << 2);
	
	// na czas testów dla Arka wyłączona konieczność przesyłania tarcia, modelu manipulatora i pidów
	pC->Jtc.jtcInitStatus = 0x00;
	
	// Check JTC init status and Joints and Gripper init status
	if(pC->Jtc.jtcInitStatus != 0x00 || pC->Jtc.jointsInitStatus != 0x00 || pC->Jtc.initStage != JTC_IS_Finish)
		pC->Jtc.initModeReq = true;			// Continue of Init state
	else
		pC->Jtc.initModeReq = false;		// Finish of Init state
}
static void Control_JtcCheckState(void)
{
	Control_JtcCheckStateError();
	Control_JtcCheckStateInit();
	Control_TrajCheckState();
	
	if(pC->Jtc.errorModeReq == true)
	{
		pC->Jtc.targetFsm = JTC_FSM_Error;
	}
	else if(pC->Jtc.initModeReq == true)
	{
		pC->Jtc.targetFsm = JTC_FSM_Init;
	}
	else if(pC->Jtc.teachingModeReq == true)
	{
		pC->Jtc.targetFsm = JTC_FSM_Teaching;
	}
	else if(pC->Jtc.teachingConstTorqueModeReq == true)
	{
		pC->Jtc.targetFsm = JTC_FSM_TeachingConstTorque;
	}
	else if(pC->Jtc.holdposModeReq == true)
	{
		pC->Jtc.targetFsm = JTC_FSM_HoldPos;
	}
	else if(pC->Jtc.operateModeReq == true)
	{
		pC->Jtc.targetFsm = JTC_FSM_Operate;
	}
	

	if(pC->Jtc.targetFsm == JTC_FSM_Error)
	{
		pC->Jtc.currentFsm = JTC_FSM_Error;
	}
	else if(pC->Jtc.targetFsm == JTC_FSM_Init)
	{
		pC->Jtc.currentFsm = JTC_FSM_Init;
	}
	else if(pC->Jtc.targetFsm == JTC_FSM_Teaching)
	{
		if(pC->Jtc.currentFsm != JTC_FSM_Teaching)
		{
			Joints_SetDefaultVariables();
			for(int num=0;num<JOINTS_MAX;num++)
			{
				pC->Joints[num].setPosTemp = 0.0;
				pC->Joints[num].setVelTemp = 0.0;
				pC->Joints[num].setAccTemp = 0.0;
			}
		}
		pC->Jtc.currentFsm = JTC_FSM_Teaching;
	}
	else if(pC->Jtc.targetFsm == JTC_FSM_TeachingConstTorque)
	{
		if(pC->Jtc.currentFsm != JTC_FSM_TeachingConstTorque)
		{
			Joints_SetDefaultVariables();
			for(int num=0;num<JOINTS_MAX;num++)
			{
				pC->Joints[num].setPosTemp = 0.0;
				pC->Joints[num].setVelTemp = 0.0;
				pC->Joints[num].setAccTemp = 0.0;
			}
		}
		pC->Jtc.currentFsm = JTC_FSM_TeachingConstTorque;
	}
	else if(pC->Jtc.targetFsm == JTC_FSM_HoldPos)
	{
		if(pC->Jtc.currentFsm != JTC_FSM_HoldPos)
		{
			Joints_SetDefaultVariables();
			for(int num=0;num<JOINTS_MAX;num++)
			{
				pC->Joints[num].setPosTemp = pC->Joints[num].currentPos; 	// aktualna pozycja jako pozycja zadana
				pC->Joints[num].setVelTemp = 0.0;
				pC->Joints[num].setAccTemp = 0.0;
			}
		}
		pC->Jtc.currentFsm = JTC_FSM_HoldPos;
	}
	else if(pC->Jtc.targetFsm == JTC_FSM_Operate)
	{
		if(pC->Jtc.currentFsm != JTC_FSM_Operate)
		{
			Joints_SetDefaultVariables();
		}
		pC->Jtc.currentFsm = JTC_FSM_Operate;
	}
}
static void Control_JtcError(void)
{
	Control_TrajClear();
	TG_ClearTgenVariables();
	Joints_SetDefaultVariables();
	pC->Jtc.teachingModeReq = false;
	
	// Reaction for internall error
	if(pC->Jtc.internalError == true)
	{
		for(uint8_t num=0;num<JOINTS_MAX;num++)
			Control_JtcSetJointToReadyToOperate(num);
	}
	
	// Reaction for externall error
	if(pC->Jtc.externalError == true)
	{
		//Waiting for host reaction
	}
}
static void Control_JtcInitPreInitStage(void)
{
	// --------- inicjalizacja grippera ---------------------------------------------------------------------------------------------------------------------------------------
	Control_JtcSetGripperToCurrentFsm();
	if(pC->Gripper.flagFirstPosRead == false)
	{
		//Czekam na odpowiedź na ramke Move z grippera. Jeżeli nie nastapi to zapewne będzie TIMEOUT na Can
	}
	else if(pC->Gripper.flagFirstPosRead == true && pC->Gripper.currentFsm == Joint_FSM_Init)
	{
		pC->Gripper.flagConfirmChangeConf = false;
		Control_JtcSetGripperToReadyToOperate();
	}
	else if(pC->Gripper.flagFirstPosRead == true && pC->Gripper.currentFsm == Joint_FSM_ReadyToOperate && pC->Gripper.flagConfirmChangeConf == false)
	{
		Control_JtcSetGripperToTargetConf();
	}
	
	// ------- podstawowa inicjalizacja jointów do momentu gdy: flagFirstPosRead = false,  flagFrictionReadFromCan = false,  Tryb = Joint_M_Torque, Fsm = Joint_FSM_ReadyToOperate -----------------
	Control_JtcReadFrictionFromJoints();
	for(uint8_t num=0;num<JOINTS_MAX;num++)
	{
		Control_JtcSetJointToCurrentFsm(num);
		
		if(pC->Joints[num].flagFirstPosRead == false)
		{
			//Czekam na odpowiedź na ramke Move z danego jointa. Jeżeli nie nastapi to zapewne będzie TIMEOUT na Can
		}
		else if(pC->Joints[num].flagFrictionReadFromCan == false)
		{
			//Czekam na odpowiedź na ramke ReadFriction z danego jointa. Jeżeli nie nastapi to zapewne będzie TIMEOUT na Can
		}
		else if(pC->Joints[num].flagFirstPosRead == true && pC->Joints[num].currentFsm == Joint_FSM_Init)
		{
			pC->Joints[num].flagConfirmChangeConf = false;
			Control_JtcSetJointToReadyToOperate(num);
		}
		else if(pC->Joints[num].flagFirstPosRead == true && pC->Joints[num].currentFsm == Joint_FSM_ReadyToOperate && pC->Joints[num].flagConfirmChangeConf == false)
		{
			Control_JtcSetJointToModeTorque(num);
		}
	}
}
static void Control_JtcInitDeparkStage(void)
{
	// Procedura deparkowania. Ruch jointem numer 1 o wartość -0.05236 rad (pozostale jointy aktualnie mają ruch o wartość 0.0). Następnie odblokowanie sworznia hamulca i potwierdzenie tego
	// Przyjęcie wartości dla ruchu
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		if(pC->Joints[num].irIsRun == false && pC->Joints[num].flagDeparkPosAchieved == false)
			Joints_StartIrValuesVariables(num, pC->Joints[num].currentPos + pC->Joints[num].deparkDist);
	
	// Realizacja ruchu na zadaną pozycje dla kazdego jointa, w celu odusnięcia od sworznia hamulca
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		if(pC->Joints[num].irIsRun == true && pC->Joints[num].flagDeparkPosAchieved == false)
			Control_JtcSetJointToEnable(num);
		
	Control_JtcPrepareSetedValuesForInit();
	RNEA_CalcTorques();
	Joints_CalcInitRegsTorque();
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		pC->Joints[num].setTorqueTemp = pC->Joints[num].idTorque + pC->Joints[num].irCurrentTorque;
	
	// Sprawdzenie osiągnięcia zadanej pozycji przy deparkowaniu. Docelowo należy to przenieść do pliku z obsluga jointów i ujednolicić robiąc jakiś sensowny regulator z rozpędzaniem
	for(int num=0;num<JOINTS_MAX;num++)
	{
		if(pC->Joints[num].irIsRun == true && pC->Joints[num].irTargetTorque < 0.0 && pC->Joints[num].currentPos < pC->Joints[num].irTargetPos)
			pC->Joints[num].flagDeparkPosAchieved = true;
		if(pC->Joints[num].irIsRun == true && pC->Joints[num].irTargetTorque > 0.0 && pC->Joints[num].currentPos > pC->Joints[num].irTargetPos)
			pC->Joints[num].flagDeparkPosAchieved = true;
		// Jeżeli dystans deparkowania bliski 0 to pomijamy faze deparkowania dla danego jointa
		if(fabs(pC->Joints[num].deparkDist) < 0.0001)
			pC->Joints[num].flagDeparkPosAchieved = true;
	}
	
	// Koniec ruchu dla danego jointa i przejście w ReadyToOperate gdy flaga flagDeparkPosAchieved == true
	// docelowo można rozszerzyć o zatrzymanie na zadanej pozycji aby nie przejechać
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		if(pC->Joints[num].flagDeparkPosAchieved == true)
		{
			Joints_StopIrValuesVariables(num);
			Control_JtcSetJointToReadyToOperate(num);
		}
}
static void Control_JtcInitRemoveBrakeStage(void)
{
	for(uint8_t num=0;num<JOINTS_MAX;num++)
	{
		Joints_StopIrValuesVariables(num);
		Control_JtcSetJointToReadyToOperate(num);
	}
	
	IO_ParkBrakeUnlock();
}
static void Control_JtcInit(void)
{
	Control_TrajClear();
	TG_ClearTgenVariables();
	Joints_SetDefaultVariables();
	pC->Jtc.teachingModeReq = false;
	for(int num=0;num<JOINTS_MAX;num++)
		pC->Joints[num].setTorqueTemp = 0.0;
	
	// możliwe fazy inicjalizacji Jtc: JTC_IS_PreInit, JTC_IS_Depark, JTC_IS_RemoveBrake, JTC_IS_Finish. Zmiana wartości zmiennej pC->Jtc.initStage w funkcji Control_JtcCheckStateInit
	if(pC->Jtc.initStage == JTC_IS_PreInit)
		Control_JtcInitPreInitStage();
	else if(pC->Jtc.initStage == JTC_IS_Depark)
		Control_JtcInitDeparkStage();
	else if(pC->Jtc.initStage == JTC_IS_RemoveBrake)
		Control_JtcInitRemoveBrakeStage();
}
static void Control_JtcTeaching(void)
{
	Control_TrajClear();
	TG_ClearTgenVariables();
	
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		if(pC->Joints[num].currentFsm != Joint_FSM_OperationEnable)
			Control_JtcSetJointToEnable(num);
		
	if(pC->Gripper.currentFsm != Joint_FSM_OperationEnable)
		Control_JtcSetGripperToEnable();
	
	Control_JtcPrepareSetedValuesForTeaching();
	RNEA_CalcTorques();
	
	for(int num=0;num<JOINTS_MAX;num++)
		pC->Joints[num].setTorqueTemp = pC->Joints[num].idTorque;
}
static void Control_JtcTeachingConstTorque(void)
{
	Control_TrajClear();
	TG_ClearTgenVariables();
	
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		if(pC->Joints[num].currentFsm != Joint_FSM_OperationEnable)
			Control_JtcSetJointToEnable(num);
		
	if(pC->Gripper.currentFsm != Joint_FSM_OperationEnable)
		Control_JtcSetGripperToEnable();
	
	Control_JtcPrepareSetedValuesForTeachingConstTorque();
	RNEA_CalcTorques();
	
	for(int num=0;num<JOINTS_MAX;num++)
		pC->Joints[num].setTorqueTemp = pC->Joints[num].idTorque + pC->Joints[num].constTorque;
}
static void Control_JtcHoldPos(void)
{
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		if(pC->Joints[num].currentFsm != Joint_FSM_OperationEnable)
			Control_JtcSetJointToEnable(num);
	if(pC->Gripper.currentFsm != Joint_FSM_OperationEnable)
		Control_JtcSetGripperToEnable();
	
	if(Traj.currentTES == TES_Stop)
		Control_TrajStop();
	else if(Traj.currentTES == TES_TransNullToStop)
		Control_TrajTransNullToStop();
	
	Control_JtcPrepareSetedValuesForHoldPos();
	Control_JtcJogAct();
	RNEA_CalcTorques();
	Joints_CalcPIDs();
	
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].setTorqueTemp = pC->Joints[num].pidTorque + pC->Joints[num].idTorque;
		pC->Joints[num].setTorqueTemp *= 1.3; //Współczynnik Arka dotyczący kompensacji strat w silniku.
	}
}
static void Control_JtcOperate(void)
{
	for(uint8_t num=0;num<JOINTS_MAX;num++)
		if(pC->Joints[num].currentFsm != Joint_FSM_OperationEnable)
			Control_JtcSetJointToEnable(num);
		
	if(pC->Gripper.currentFsm != Joint_FSM_OperationEnable)
		Control_JtcSetGripperToEnable();
	
	Traj.numInterPoint++;
	if(Traj.numInterPoint >= Traj.maxInterPoints)
	{
		Traj.targetTES = TES_Finish; // Koniec trajektorii
		return;
	}
	
	Control_TrajInterpolate();
	Control_JtcPrepareSetedValuesForOperate();
	RNEA_CalcTorques();
	Joints_CalcPIDs();
	Joints_CalcFrictionCompensate();
	
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].setTorqueTemp = pC->Joints[num].pidTorque + pC->Joints[num].idTorque + pC->Joints[num].fricTorque;
		pC->Joints[num].setTorqueTemp *= 1.3; //Współczynnik Arka dotyczący kompensacji strat w silniku.
	}
}
static void Control_JtcAct(void)
{
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	
	IO_InputsAct();
	Control_CheckErrorFlags();
	Control_JtcCheckState();
	Control_JtcChangeTool();
	Kin_RobPosAct();
	
	if(pC->Jtc.currentFsm == JTC_FSM_Error)
	{
		LED3_ON;
		Control_JtcError();
	}
	else if(pC->Jtc.currentFsm == JTC_FSM_Init)
	{
		LED1_ON;
		Control_JtcInit();
	}
	else if(pC->Jtc.currentFsm == JTC_FSM_Teaching)
	{
		LED1_ON;
		LED2_ON;
		Control_JtcTeaching();
	}
	else if(pC->Jtc.currentFsm == JTC_FSM_TeachingConstTorque)
	{
		LED1_ON;
		LED2_ON;
		Control_JtcTeachingConstTorque();
	}
	else if(pC->Jtc.currentFsm == JTC_FSM_HoldPos)
	{
		LED2_ON;
		Control_JtcHoldPos();
	}
	else if(pC->Jtc.currentFsm == JTC_FSM_Operate)
	{
		LED2_ON;
		Control_JtcOperate();
	}
	
	Control_SetVariableForIgnoreDevices();
	Control_CheckLimits();
	Control_CheckErrorFlags();
	Control_SetNewTorqueValues();
	pC->Can.TxMsgs[Can_TxF_Move].reqSend = true;
	Control_SendCommandClearErrorsToJoints();
	Control_SendCommandResetDevice();
	IO_OutputsAct();
	Control_SendDataToJoints();
	MBS_Act();
	#ifdef DEBUG
	Debug_PrepareFrame();
	#endif
	#ifdef MATLABSIM
	MatlabSim_SendFrame();
	#endif
}
// ********************** Interrupts functions ***********************************
void TIM7_IRQHandler(void)
{
	if((TIM7->SR & TIM_SR_UIF) != RESET)
	{
		Control_JtcAct();
		TIM7->SR &= ~TIM_SR_UIF;
	}
}
void SysTick_Handler(void)
{
	pC->tick++;
}
