#include "Joints.h"
extern sControl* pC;
void Joints_SetDefaultFriction(void)
{
	//J1
	pC->Joints[0].fricCoeff[0] = 8.3511;
	pC->Joints[0].fricCoeff[1] = 49.0592;
	pC->Joints[0].fricCoeff[2] = -33.8717;
	pC->Joints[0].fricCoeff[3] = 14.5622;
	pC->Joints[0].fricCoeff[4] = -0.016903;
	pC->Joints[0].fricCoeff[5] = 14.01489;
	
	pC->Joints[1].fricCoeff[0] = 6.524288;
	pC->Joints[1].fricCoeff[1] = 46.54229369287933;
	pC->Joints[1].fricCoeff[2] = -37.98456500960326;
	pC->Joints[1].fricCoeff[3] = 21.0199476124962;
	pC->Joints[1].fricCoeff[4] = -0.027194946468284284;
	pC->Joints[1].fricCoeff[5] = 19.88786510460206;
	
	pC->Joints[2].fricCoeff[0] = 5.906889543057651;
	pC->Joints[2].fricCoeff[1] = 46.74362364518938;
	pC->Joints[2].fricCoeff[2] = -51.63074061579354;
	pC->Joints[2].fricCoeff[3] = 37.849346936033356;
	pC->Joints[2].fricCoeff[4] = -0.02898496744241571;
	pC->Joints[2].fricCoeff[5] = 24.734728760729638;
	
	pC->Joints[3].fricCoeff[0] = 2.6687306340217645;
	pC->Joints[3].fricCoeff[1] = 12.573326515302712;
	pC->Joints[3].fricCoeff[2] = -3.716488061498839;
	pC->Joints[3].fricCoeff[3] = -0.6543677283056056;
	pC->Joints[3].fricCoeff[4] = -0.023931660300091872;
	pC->Joints[3].fricCoeff[5] = 25.619160826675138;
	
	pC->Joints[4].fricCoeff[0] = 2.888895376338052;
	pC->Joints[4].fricCoeff[1] = 18.96976439814303;
	pC->Joints[4].fricCoeff[2] = -22.293583023278945;
	pC->Joints[4].fricCoeff[3] = 16.941868944179486;
	pC->Joints[4].fricCoeff[4] = -0.018687413997168847;
	pC->Joints[4].fricCoeff[5] = 31.076305032122963;
	
	pC->Joints[5].fricCoeff[0] = 3.0290731310886185;
	pC->Joints[5].fricCoeff[1] = 21.98125488265331;
	pC->Joints[5].fricCoeff[2] = -20.186097814102737;
	pC->Joints[5].fricCoeff[3] = 12.648620008379257;
	pC->Joints[5].fricCoeff[4] = -0.02301637403516809;
	pC->Joints[5].fricCoeff[5] = 19.112761551382018;
	
	//J2
//	pC->Joints[0].fricCoeff[0] = 3.7557695071018817;
//	pC->Joints[0].fricCoeff[1] = 32.88390062972518;
//	pC->Joints[0].fricCoeff[2] = -45.42756793785036;
//	pC->Joints[0].fricCoeff[3] = 36.71164726130435;
//	pC->Joints[0].fricCoeff[4] = -0.07448704781806412;
//	pC->Joints[0].fricCoeff[5] = 37.274508595534286;
//	
//	pC->Joints[1].fricCoeff[0] = 4.93588;
//	pC->Joints[1].fricCoeff[1] = 38.31554;
//	pC->Joints[1].fricCoeff[2] = -36.30966;
//	pC->Joints[1].fricCoeff[3] = 22.156883;
//	pC->Joints[1].fricCoeff[4] = -0.03728;
//	pC->Joints[1].fricCoeff[5] = 29.39539;
//	
//	pC->Joints[2].fricCoeff[0] = 5.121409854381762;
//	pC->Joints[2].fricCoeff[1] = 36.90630225186322;
//	pC->Joints[2].fricCoeff[2] = -33.77905899259529;
//	pC->Joints[2].fricCoeff[3] = 19.092281381375987;
//	pC->Joints[2].fricCoeff[4] = -0.03338800832321824;
//	pC->Joints[2].fricCoeff[5] = 28.75503460003772;
//	
//	pC->Joints[3].fricCoeff[0] = 3.634361182476341;
//	pC->Joints[3].fricCoeff[1] = 16.367108106695177;
//	pC->Joints[3].fricCoeff[2] = -7.535778971936864;
//	pC->Joints[3].fricCoeff[3] = 0.4903211032620818;
//	pC->Joints[3].fricCoeff[4] = -0.0376143000898514;
//	pC->Joints[3].fricCoeff[5] = 20.98337065936175;
//	
//	pC->Joints[4].fricCoeff[0] = 2.644900549836491;
//	pC->Joints[4].fricCoeff[1] = 13.470815934063692;
//	pC->Joints[4].fricCoeff[2] = -11.533519586705612;
//	pC->Joints[4].fricCoeff[3] = 6.1002408416911305;
//	pC->Joints[4].fricCoeff[4] = -0.03576808486611048;
//	pC->Joints[4].fricCoeff[5] = 18.98088635678911;
//	
//	pC->Joints[5].fricCoeff[0] = 2.4254685647398695;
//	pC->Joints[5].fricCoeff[1] = 15.749754158317014;
//	pC->Joints[5].fricCoeff[2] = -12.106454476243666;
//	pC->Joints[5].fricCoeff[3] = 5.493472586452902;
//	pC->Joints[5].fricCoeff[4] = -0.024242620835343617;
//	pC->Joints[5].fricCoeff[5] = 19.044821862593533;
}
void Joints_SetFrictionFromCan(uint8_t num)
{
	for(int i=0;i<JOINTS_FRICCOEFFMAX;i++)
	{
		pC->Joints[num].fricCoeff[i] = pC->Joints[num].fricCoeffFromCan[i];
	}
}
void Joints_SetDefaultPidParam(void)
{
	pC->Joints[0].pidKp = 8000.0;
	pC->Joints[0].pidKi = 14000.0;
	pC->Joints[0].pidKd = 160.0;
	pC->Joints[0].pidErrorIntMin = -120.0;
	pC->Joints[0].pidErrorIntMax = 120.0;
	
	pC->Joints[1].pidKp = 8000.0;
	pC->Joints[1].pidKi = 14000.0;
	pC->Joints[1].pidKd = 160.0;
	pC->Joints[1].pidErrorIntMin = -100.0;
	pC->Joints[1].pidErrorIntMax = 100.0;
	
	pC->Joints[2].pidKp = 8000.0;
	pC->Joints[2].pidKi = 14000.0;
	pC->Joints[2].pidKd = 160.0;
	pC->Joints[2].pidErrorIntMin = -80.0;
	pC->Joints[2].pidErrorIntMax = 80.0;
	
	pC->Joints[3].pidKp = 4000.0;
	pC->Joints[3].pidKi = 24000.0;
	pC->Joints[3].pidKd = 100.0;
	pC->Joints[3].pidErrorIntMin = -40.0;
	pC->Joints[3].pidErrorIntMax = 40.0;
	
	pC->Joints[4].pidKp = 4000.0;
	pC->Joints[4].pidKi = 24000.0;
	pC->Joints[4].pidKd = 100.0;
	pC->Joints[4].pidErrorIntMin = -40.0;
	pC->Joints[4].pidErrorIntMax = 40.0;
	
	pC->Joints[5].pidKp = 4000.0;
	pC->Joints[5].pidKi = 24000.0;
	pC->Joints[5].pidKd = 100.0;
	pC->Joints[5].pidErrorIntMin = -40.0;
	pC->Joints[5].pidErrorIntMax = 40.0;
}
void Joints_SetStartValuesVariables(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].flagFirstPosRead = false;
		pC->Joints[num].flagConfirmChangeConf = false;
		pC->Joints[num].currentMode = Joint_M_Null;
		pC->Joints[num].targetMode = Joint_M_Torque;
		pC->Joints[num].confFun = 0x07; //[0x01 - wlaczenie ograniczenia zakresu pracy, 0x02 - wlaczenie MA730, 0x04 - obsluga safety]
		
		pC->Joints[num].flagDeparkPosAchieved = false;
		pC->Joints[num].deparkDist = 0.0;
		
		pC->Joints[num].setPos = 0.0;
		pC->Joints[num].setVel = 0.0;
		pC->Joints[num].setAcc = 0.0;
		pC->Joints[num].setTorque = 0.0;
		pC->Joints[num].constTorque = 0.0;
		
		pC->Joints[num].limitPosMin = -M_PI;
		pC->Joints[num].limitPosMax = M_PI;
		pC->Joints[num].limitVelMin = -2.0;
		pC->Joints[num].limitVelMax = 2.0;
		pC->Joints[num].limitAccMin = -M_4_PI;
		pC->Joints[num].limitAccMax = M_4_PI;
		pC->Joints[num].limitTorqueMin = -360.0;
		pC->Joints[num].limitTorqueMax = 360.0;
		pC->Joints[num].limitTempMin = 0.0;
		pC->Joints[num].limitTempMax = 255.0;
		pC->Joints[num].limitPosErrorMin = -0.09;
		pC->Joints[num].limitPosErrorMax = 0.09;
		
		pC->Joints[num].maxPosCom = M_PI;
		pC->Joints[num].maxVelCom = 2.0;
		pC->Joints[num].maxAccCom = M_4_PI;
		pC->Joints[num].maxTorqueCom = 360.0;
		
		pC->Joints[num].fricTorque = 0.0;
		
		pC->Joints[num].idSetPos = 0.0;
		pC->Joints[num].idSetVel = 0.0;
		pC->Joints[num].idSetAcc = 0.0;
		pC->Joints[num].idTorque = 0.0;
		
		pC->Joints[num].pidDt = 0.001;
		pC->Joints[num].pidErrorCurrent = 0.0;
		pC->Joints[num].pidErrorMeanCurrent = 0.0;
		pC->Joints[num].pidErrorMeanPrev = 0.0;
		pC->Joints[num].pidErrorBufIdx = 0;
		for(uint32_t i=0;i<JOINTS_PIDBUFMAX;i++)
			pC->Joints[num].pidErrorBuf[i] = 0.0;
		pC->Joints[num].pidErrorDiv = 0.0;
		pC->Joints[num].pidErrorInt = 0.0;
		pC->Joints[num].pidTorque = 0.0;
		
		pC->Joints[num].irIsRun = false;
		pC->Joints[num].irMaxTorque = 20.0;
		pC->Joints[num].irDt = 0.001;
		pC->Joints[num].irCurrentTorque = 0.0;
		pC->Joints[num].irTargetTorque = 0.0;
		pC->Joints[num].irErrorTorque = 0.0;
		pC->Joints[num].irHyst = 0.2;
		pC->Joints[num].irRampTorque = 1.0;	//Unit: Nm/sek
	}
	
	pC->Joints[1].deparkDist = -0.05236;
}
void Joints_SetResetValuesVariables(uint8_t num)
{
	pC->Joints[num].flagFirstPosRead = false;
	pC->Joints[num].flagConfirmChangeConf = false;
	pC->Joints[num].currentMode = Joint_M_Null;
	pC->Joints[num].targetMode = Joint_M_Torque;
	pC->Joints[num].currentFsm = Joint_FSM_Start;
	pC->Joints[num].targetFsm = Joint_FSM_Init;
	
	pC->Joints[num].setPos = 0.0;
	pC->Joints[num].setVel = 0.0;
	pC->Joints[num].setAcc = 0.0;
	pC->Joints[num].setTorque = 0.0;
	
	pC->Joints[num].fricTorque = 0.0;
	
	pC->Joints[num].idSetPos = 0.0;
	pC->Joints[num].idSetVel = 0.0;
	pC->Joints[num].idSetAcc = 0.0;
	pC->Joints[num].idTorque = 0.0;
	
	pC->Joints[num].pidErrorCurrent = 0.0;
	pC->Joints[num].pidErrorMeanCurrent = 0.0;
	pC->Joints[num].pidErrorMeanPrev = 0.0;
	pC->Joints[num].pidErrorBufIdx = 0;
	for(uint32_t i=0;i<JOINTS_PIDBUFMAX;i++)
		pC->Joints[num].pidErrorBuf[i] = 0.0;
	pC->Joints[num].pidErrorDiv = 0.0;
	pC->Joints[num].pidErrorInt = 0.0;
	pC->Joints[num].pidTorque = 0.0;
	
	pC->Joints[num].irIsRun = false;
	pC->Joints[num].irCurrentTorque = 0.0;
	pC->Joints[num].irTargetTorque = 0.0;
	pC->Joints[num].irErrorTorque = 0.0;
}
void Joints_StartIrValuesVariables(uint8_t num, double targetPos)
{
	pC->Joints[num].irIsRun = true;
	
	pC->Joints[num].irTargetPos = targetPos;
	
	if(pC->Joints[num].currentPos > pC->Joints[num].irTargetPos)
		pC->Joints[num].irTargetTorque = -pC->Joints[num].irMaxTorque;
	else
		pC->Joints[num].irTargetTorque = pC->Joints[num].irMaxTorque;
	
	pC->Joints[num].irCurrentTorque = 0.0;
	pC->Joints[num].irErrorTorque = 0.0;
}
void Joints_StopIrValuesVariables(uint8_t num)
{
	pC->Joints[num].irIsRun = false;
	pC->Joints[num].irTargetTorque = 0.0;
	pC->Joints[num].irCurrentTorque = 0.0;
	pC->Joints[num].irErrorTorque = 0.0;
}
void Joints_SetDefaultVariables(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		pC->Joints[num].setPos = 0.0;
		pC->Joints[num].setVel = 0.0;
		pC->Joints[num].setAcc = 0.0;
		pC->Joints[num].setTorque = 0.0;
		pC->Joints[num].constTorque = 0.0;
		
		pC->Joints[num].fricTorque = 0.0;
		
		pC->Joints[num].idSetPos = 0.0;
		pC->Joints[num].idSetVel = 0.0;
		pC->Joints[num].idSetAcc = 0.0;
		pC->Joints[num].idTorque = 0.0;
		
		pC->Joints[num].pidErrorCurrent = 0.0;
		pC->Joints[num].pidErrorMeanCurrent = 0.0;
		pC->Joints[num].pidErrorMeanPrev = 0.0;
		pC->Joints[num].pidErrorBufIdx = 0;
		for(uint32_t i=0;i<JOINTS_PIDBUFMAX;i++)
			pC->Joints[num].pidErrorBuf[i] = 0.0;
		pC->Joints[num].pidErrorDiv = 0.0;
		pC->Joints[num].pidErrorInt = 0.0;
		pC->Joints[num].pidTorque = 0.0;
	}
}
void Joints_CalcFrictionCompensate(void)
{
	for(uint8_t num=0;num<JOINTS_MAX;num++)
	{
		double vel = pC->Joints[num].setVelTemp;
		double t = pC->Joints[num].currentBearingTemp;
		double coeffs[JOINTS_FRICCOEFFMAX];
		for(int i=0;i<JOINTS_FRICCOEFFMAX;i++)
			coeffs[i] = pC->Joints[num].fricCoeff[i];
			
		pC->Joints[num].fricTorque = (1+coeffs[4]*(t-coeffs[5]))*(coeffs[0] * ((vel > 0) - (vel < 0)) + coeffs[1] * vel + coeffs[2] * pow(vel, 2) * ((vel > 0) - (vel < 0)) + coeffs[3] * pow(vel, 3));
	}
}
void Joints_CalcPIDs(void)
{
	// num = numer jointa
	// pidDt - podstawa czasu = 0.001 (s)
	// setPos - pozycja zadana (rad)
	// currentPos - pozycja aktualna (rad)
	// pidErrorPrev - uchyb poprzedni (rad)
	// pidErrorCurrent - uchyb aktualny (rad)
	// pidErrorDiv - pochodna uchybu (rad/s)
	// pidErrorInt - calka uchybu (rad*s)
	// pidErrorIntMin - minimalna wartosc calki uchybu (do saturacji calki) (rad*s)
	// pidErrorIntMax - maksymalna wartosc calki uchybu (do saturacji calki) (rad*s)
	// pidOut - wyjscie regulatora (Nm)
	// pidKp - wspólczynnik kp (Nm/rad)
	// pidKi - wspólczynnik ki (Nm/(rad*s))
	// pidKd - wspólczynnik kd (Nm/(rad/s))
	
	for(int num=0;num<JOINTS_MAX;num++)
	{
		// uchyb regulacji
		pC->Joints[num].pidErrorCurrent = pC->Joints[num].setPosTemp - pC->Joints[num].currentPos;

		// pochodna uchybu liczona jakos wartosc srednia z uchybów z ostatnich JOINTS_PIDBUFMAX kroków czasowych
		pC->Joints[num].pidErrorBuf[pC->Joints[num].pidErrorBufIdx] = pC->Joints[num].pidErrorCurrent;
		pC->Joints[num].pidErrorBufIdx = (pC->Joints[num].pidErrorBufIdx + 1) % JOINTS_PIDBUFMAX;
		pC->Joints[num].pidErrorMeanPrev = pC->Joints[num].pidErrorMeanCurrent;
		pC->Joints[num].pidErrorMeanCurrent = 0.0;
		for(uint32_t j=0;j<JOINTS_PIDBUFMAX;j++)
			pC->Joints[num].pidErrorMeanCurrent += pC->Joints[num].pidErrorBuf[j];
		pC->Joints[num].pidErrorMeanCurrent /= (double)JOINTS_PIDBUFMAX;
		pC->Joints[num].pidErrorDiv = (pC->Joints[num].pidErrorMeanCurrent - pC->Joints[num].pidErrorMeanPrev) / pC->Joints[num].pidDt; 
		
		//calka uchybu i saturacja calki uchybu
		pC->Joints[num].pidErrorInt += pC->Joints[num].pidErrorCurrent * pC->Joints[num].pidDt;	
		if(pC->Joints[num].pidErrorInt < pC->Joints[num].pidErrorIntMin) 
			pC->Joints[num].pidErrorInt = pC->Joints[num].pidErrorIntMin;
		if(pC->Joints[num].pidErrorInt > pC->Joints[num].pidErrorIntMax)
			pC->Joints[num].pidErrorInt = pC->Joints[num].pidErrorIntMax;
		
		//wyjscie z regulatora
		pC->Joints[num].pidTorqueP = pC->Joints[num].pidKp * pC->Joints[num].pidErrorCurrent;
		pC->Joints[num].pidTorqueI = pC->Joints[num].pidKi * pC->Joints[num].pidErrorInt;
		pC->Joints[num].pidTorqueD = pC->Joints[num].pidKd * pC->Joints[num].pidErrorDiv;
		pC->Joints[num].pidTorque = pC->Joints[num].pidTorqueP + pC->Joints[num].pidTorqueI + pC->Joints[num].pidTorqueD;
	}
}
void Joints_CalcInitRegsTorque(void)
{
	for(int num=0;num<JOINTS_MAX;num++)
	{
		if(pC->Joints[num].irIsRun == true)
		{
			pC->Joints[num].irErrorTorque = pC->Joints[num].irTargetTorque - pC->Joints[num].irCurrentTorque;
			
			if(pC->Joints[num].irErrorTorque < -pC->Joints[num].irHyst)
				pC->Joints[num].irCurrentTorque -= pC->Joints[num].irRampTorque * pC->Joints[num].irDt;
			if(pC->Joints[num].irErrorTorque > pC->Joints[num].irHyst)
				pC->Joints[num].irCurrentTorque += pC->Joints[num].irRampTorque * pC->Joints[num].irDt;
		}
		else
		{
			Joints_StopIrValuesVariables(num);
		}
	}
}
