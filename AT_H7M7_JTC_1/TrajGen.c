#include "TrajGen.h"
extern sControl* pC;
extern sTrajectory Traj;
extern sMB_RTUSlave	Mbs;
double accmax = 10.0; //minimum 1.0, maximum 4*pi
const double prec = 0.000001; //precyzja obliczen
union conv32
{
    uint32_t u32; // here_write_bits
    float    f32; // here_read_float
};
union conv64
{
    uint64_t u64; // here_write_bits
    double   d64; // here_read_double
};
static double _Complex cmplDiv(double _Complex z1, double _Complex z2)
{
	//Wlasna funkcja do dzielenia liczb zespolonych. Keil, mimo uzycia C99, wyrzuca blad "undefined symbol __divdc3" przy próbie dzielenia licz zespolonych
	double _Complex z3;
	double a = creal(z1), b = cimag(z1);
	double c = creal(z2), d = cimag(z2);
	double z3r = (a*c+b*d)/(pow(c,2) + pow(d,2));
	double z3i = (b*c-a*d)/(pow(c,2)+ pow(d,2));
	z3 = z3r + z3i * I;
	return z3;
}
void TG_SetDefaultVariables(void)
{
	Traj.Tgen.stepTime = 0.001;
	Traj.Tgen.seqNum = 0;
	Traj.Tgen.maxwaypoints = 0;
	Traj.Tgen.maxpoints = 0;
	Traj.Tgen.reqTrajPrepare = false;
	Traj.Tgen.minVelocity = 0.1; //Unit: rad/sek
	
	if(accmax < 1.0)
		accmax = 1.0;
	if(accmax > M_4_PI)
		accmax = M_4_PI;
	
	
	//Znajdujemy najmniejszy z górnych limitów predkosci jointów i te wartosc przyjmujemy jako maksymalna dla ruchu w trajektorii Unit: rad/sek
	Traj.Tgen.maxVelocity = fabs(pC->Joints[0].limitVelMax);
	for(int num=0;num<JOINTS_MAX;num++)
		if(fabs(pC->Joints[num].limitVelMax) < Traj.Tgen.maxVelocity)
			Traj.Tgen.maxVelocity = fabs(pC->Joints[num].limitVelMax);
	
	for(int num=0;num<JOINTS_MAX;num++)
		for(int i=0;i<TG_SEQWAYPOINTSSMAX;i++)
		{
			Traj.Tgen.path[num][i].q0 = 0.0;
			Traj.Tgen.path[num][i].q1 = 0.0;
			Traj.Tgen.path[num][i].v0 = 0.0;
			Traj.Tgen.path[num][i].v1 = 0.0;
			Traj.Tgen.path[num][i].t1 = 0.0;
			Traj.Tgen.path[num][i].vmax = 0.0;
			Traj.Tgen.path[num][i].amax = 0.0;
			Traj.Tgen.path[num][i].dir = 0;
			Traj.Tgen.path[num][i].n = 0;
		}

	for(int i=0;i<TG_SEQWAYPOINTSSMAX;i++)
	{
		Traj.Tgen.waypoints[i].active = false;
		Traj.Tgen.waypoints[i].type = SPT_Finish;
		Traj.Tgen.waypoints[i].moveType = SPMT_Null;
		Traj.Tgen.waypoints[i].vel = 0.0;
		Traj.Tgen.waypoints[i].zone = 0.0;
		Traj.Tgen.waypoints[i].refSystem = 0;
		
		Traj.Tgen.waypoints[i].pos = Vec6Zeros();
		Traj.Tgen.waypoints[i].qSol = Vec6Zeros();
		Traj.Tgen.waypoints[i].pos = Vec6Zeros();
		Traj.Tgen.waypoints[i].quat = Vec4Zeros();
		Traj.Tgen.waypoints[i].mat = Mat4Ones();
		for(int j=0;j<IK_SOLNUMREAL;j++)
		{
			Traj.Tgen.waypoints[i].sol.v[j] = Vec6Zeros();
		}
	}
	Traj.Tgen.status = TGS_Idle;
}
void TG_ClearTgenVariables(void)
{
	if(Traj.Tgen.status == TGS_Idle)
		return;
	
	Traj.Tgen.stepTime = 0.001;
	Traj.Tgen.seqNum = 0;
	Traj.Tgen.recwaypoints = 0;
	Traj.Tgen.maxwaypoints = 0;
	Traj.Tgen.maxpoints = 0;
	Traj.Tgen.reqTrajPrepare = false;
	
	Traj.Tgen.status = TGS_Idle;
}
void TG_Conf(void)
{
	TG_SetDefaultVariables();
}
static sRobPos TG_GetSeqPointConfSpace(uint16_t numidx)
{
	uint16_t idx = numidx;
	sRobPos p;
	union conv32 x;
	p.moveType = (eSeqPointMoveType)Mbs.hregs[idx++];
	for(uint32_t j=0;j<JOINTS_MAX;j++)
	{
		x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
		x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
		p.pos.v[j] = x.f32;
	}
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.vel = x.f32;
	p.type = SPT_Way;
	
	return p;
}
static sRobPos TG_GetSeqPointKartesianSpace(uint16_t numidx)
{
	uint16_t idx = numidx;
	sRobPos p;
	p.mat = Mat4Ones();
	union conv32 x;
	p.moveType = (eSeqPointMoveType)Mbs.hregs[idx++];
	
	//Wektor pozycji XYZ
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.mat.v[0][3] = x.f32;
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.mat.v[1][3] = x.f32;
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.mat.v[2][3] = x.f32;
	
	//Wektor orientacji w kwaternionach
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.quat.v[0] = x.f32;
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.quat.v[1] = x.f32;
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.quat.v[2] = x.f32;
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.quat.v[3] = x.f32;
	
	//Wektor konfiguracji rozwiazania kinematyki
	p.conf.v[0] = Mbs.hregs[idx++];
	p.conf.v[1] = Mbs.hregs[idx++];
	p.conf.v[2] = Mbs.hregs[idx++];
	p.conf.v[3] = Mbs.hregs[idx++];
	
	//Numer ukladu wspólrzednych wzgledem którego jest podawana pozycja i orientacja: 0 - uklad bazowy robota
	p.refSystem = Mbs.hregs[idx++];
	
	//Predkosc maksymalna dla tego punktu
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.vel = x.f32;
	
	//Promien okregu w jakim ma zmiescic sie robot przy omijaniu waypointów. Wartosc 0.0 oznacza brak omijania w tym punkcie.
	x.u32 = (uint32_t)Mbs.hregs[idx++] << 16;
	x.u32 += (uint32_t)Mbs.hregs[idx++] << 0;
	p.zone = x.f32;
	
	p = Kin_IKCalcFromQuat(p);
	p = Kin_FindNearestSolution(p);
	p.pos = p.qSol;
	
	p.type = SPT_Way;
	
	return p;
}
static bool TG_GetSeqFromMbs(void)
{
	uint32_t idx = MRN_SeqStart;
	Traj.Tgen.seqNum = Mbs.hregs[idx++];
	Traj.Tgen.recwaypoints = Mbs.hregs[idx++];
	Traj.Tgen.maxwaypoints = Traj.Tgen.recwaypoints + 1;
	Traj.Tgen.maxpathpoints = Traj.Tgen.recwaypoints;
	
	if(Traj.Tgen.recwaypoints == 0)
	{
		Traj.Tgen.trajPrepStatus = TPS_NotEnoughWaypoints;
		return false;
	}
	
	if(Traj.Tgen.maxwaypoints > TG_SEQWAYPOINTSSMAX)
	{
		Traj.Tgen.trajPrepStatus = TPS_ToMuchWaypoints;
		return false;
	}
	
	//Waypoint zerowy - aktualna pozycja
	for(uint32_t j=0;j<JOINTS_MAX;j++)
		Traj.Tgen.waypoints[0].pos.v[j] = pC->Joints[j].currentPos;
	Traj.Tgen.waypoints[0].vel = 0.0;
	Traj.Tgen.waypoints[0].type = SPT_Start;
	Traj.Tgen.waypoints[0].moveType = SPMT_ConfSpacePtp;
	
	//wayointy odebrane z modbus
	for(uint32_t i=1;i<Traj.Tgen.maxwaypoints;i++)
	{
		if((eSeqPointMoveType)Mbs.hregs[idx] == SPMT_ConfSpacePtp)
		{
			Traj.Tgen.waypoints[i] = TG_GetSeqPointConfSpace(idx);
			idx += 15;
		}
		else if((eSeqPointMoveType)Mbs.hregs[idx] == SPMT_KartSpacePtp)
		{
			Traj.Tgen.waypoints[i] = TG_GetSeqPointKartesianSpace(idx);
			idx += 24;
		}
	}
	//ostatni waypoint ma status SPT_Finish
	Traj.Tgen.waypoints[Traj.Tgen.maxwaypoints-1].type = SPT_Finish;
	
	
	//Sprawdzenie zadanych pozycji i predkosci pod katem limitów
	for(uint32_t i=0;i<Traj.Tgen.maxwaypoints;i++)
	{
		for(uint32_t j=0;j<JOINTS_MAX;j++)
		{
			if(Traj.Tgen.waypoints[i].pos.v[j] < pC->Joints[j].limitPosMin)
			{
				Traj.Tgen.trajPrepStatus = TPS_PosToLow;
				return false;
			}
			if(Traj.Tgen.waypoints[i].pos.v[j] > pC->Joints[j].limitPosMax)
			{
				Traj.Tgen.trajPrepStatus = TPS_PosToHigh;
				return false;
			}
		}
		
		if((i != 0) && (fabs(Traj.Tgen.waypoints[i].vel) < Traj.Tgen.minVelocity)) //nie sprawdzam predkosci dla punktu startowego
		{
			Traj.Tgen.trajPrepStatus = TPS_VelocityToLow;
			return false;
		}
		if(fabs(Traj.Tgen.waypoints[i].vel) > Traj.Tgen.maxVelocity)
		{
			Traj.Tgen.trajPrepStatus = TPS_VelocityToHigh;
			return false;
		}
	}
	Traj.Tgen.waypoints[Traj.Tgen.maxwaypoints].type = SPT_Finish;
	return true;
}
// ******************************** Polynomial 3rd order ****************************************
static bool TG_POLY3_VarIsReal(double v)
{
	char buf[10];
	sprintf(buf, "%f", v);
	if(buf[0] == 'n'|| buf[1] == 'n'|| buf[2] == 'n')
		return false;
	return true;
}
static double TG_POLY3_q(double q0, double q1, double v0, double v1, double t1, double t)
{
	return q0+t*v0+(pow(t,3)*(2*q0-2*q1+t1*(v0+v1)))/pow(t1,3)-(pow(t,2)*(3*q0-3*q1+t1*(2*v0+v1)))/pow(t1,2);
}
static double TG_POLY3_dq(double q0, double q1, double v0, double v1, double t1, double t)
{
	return ((t-t1)*(6*q0*t-6*q1*t+(3*t-t1)*t1*v0)+t*(3*t-2*t1)*t1*v1)/pow(t1,3);
}
static double TG_POLY3_ddq(double q0, double q1, double v0, double v1, double t1, double t)
{
	return (2*(q0*(6*t-3*t1)+3*q1*(-2*t+t1)+t1*(3*t*(v0+v1)-t1*(2*v0+v1))))/pow(t1,3);
}
static double TG_POLY3_FindTendFromAcc(double q0, double q1, double v0, double v1, double t1, double vma, double amax)
{
	double tend[8];
	tend[0] = (2*v0+v1-sqrt(6*amax*(q0-q1)+pow(2*v0+v1,2)))/amax;
	tend[1] = (2*v0+v1+sqrt(6*amax*(q0-q1)+pow(2*v0+v1,2)))/amax;
	tend[2] = -((2*v0+v1+sqrt(-6*amax*(q0-q1)+pow(2*v0+v1,2)))/amax);
	tend[3] = (-2*v0-v1+sqrt(-6*amax*(q0-q1)+pow(2*v0+v1,2)))/amax;
	tend[4] = (v0+2*v1-sqrt(6*amax*(q0-q1)+pow(v0+2*v1,2)))/amax;
	tend[5] = (v0+2*v1+sqrt(6*amax*(q0-q1)+pow(v0+2*v1,2)))/amax;
	tend[6] = -((v0+2*v1+sqrt(-6*amax*(q0-q1)+pow(v0+2*v1,2)))/amax);
	tend[7] = (-v0-2*v1+sqrt(-6*amax*(q0-q1)+pow(v0+2*v1,2)))/amax;
	
	double maxval;
	for(int i=0;i<8;i++)
	{
		if(TG_POLY3_VarIsReal(tend[i]))
		{
			maxval = tend[i];
			break;
		}
	}
	
	for(int i=0;i<8;i++)
	{
		if(TG_POLY3_VarIsReal(tend[i]) && tend[i] > maxval)
		{
			maxval = tend[i];
		}
	}
	
	return maxval;
}
static double TG_POLY3_FindTendFromVel(double q0, double q1, double v0, double v1, double t1, double vma, double amax)
{
	//do obliczen v0 musi byc rózne od zera
	if(fabs(v0) < prec)
		v0 = prec;
	
	double t1a = (3*sqrt(pow(q0-q1,2)*(v0-vma)*(v1-vma))-3*(q0-q1)*(v0+v1+vma))/(pow(v0,2)+v0*v1+pow(v1,2)+3*(v0+v1)*vma);
	double t1b = (-3*sqrt(pow(q0-q1,2)*(v0-vma)*(v1-vma))-3*(q0-q1)*(v0+v1+vma))/(pow(v0,2)+v0*v1+pow(v1,2)+3*(v0+v1)*vma);
	
	double qvma1a = -(pow(3*q0-3*q1+t1a*v0,2)+t1a*(6*q0-6*q1+t1a*v0)*v1+pow(t1a,2)*pow(v1,2))/(3.*t1a*(2*q0-2*q1+t1a*(v0+v1)));
	double qvma1b = -(pow(3*q0-3*q1+t1b*v0,2)+t1b*(6*q0-6*q1+t1b*v0)*v1+pow(t1b,2)*pow(v1,2))/(3.*t1b*(2*q0-2*q1+t1b*(v0+v1)));
	
	if(t1a < 0.0 && t1b < 0.0)						t1 = fabs(t1a);
	if(fabs(qvma1a - vma) < prec)					t1 = fabs(t1a);
	if(fabs(qvma1b - vma) < prec)					t1 = fabs(t1b);
	if(t1a > 0.0 && fabs(qvma1a - vma) < prec)		t1 = fabs(t1a);
	if(t1b > 0.0 && fabs(qvma1b - vma) < prec)		t1 = fabs(t1b);
	
	return t1;
}
static double TG_POLY3_FindMaxvelFromTend(double q0, double q1, double v0, double v1, double t1, double vma, double amax)
{
	return -(pow(3*q0-3*q1+t1*v0,2)+t1*(6*q0-6*q1+t1*v0)*v1+pow(t1,2)*pow(v1,2))/(3.*t1*(2*q0-2*q1+t1*(v0+v1)));
}
static double TG_POLY3_FindOnePath_txa(double q1, double tt1, double qv)
{
	double _Complex z1 = tt1/2.0;
	double _Complex z2 = -cmplDiv((q1*cpow(tt1,2)),(2.*cpow(-(cpow(q1,2)*(q1 - 2*qv)*cpow(tt1,3)) + 2*csqrt(-(cpow(q1,4)*(q1 - qv)*qv*cpow(tt1,6))),0.3333333333333333)));
	double _Complex z3 = -cpow(-(cpow(q1,2)*(q1 - 2*qv)*cpow(tt1,3)) + 2*csqrt(-(cpow(q1,4)*(q1 - qv)*qv*cpow(tt1,6))),0.3333333333333333)/(2.*q1);

	return creal(z1+z2+z3);
}
static double TG_POLY3_FindOnePath_txb(double q1, double tt1, double qv)
{
	double _Complex z0a = (0+0.25*I);
	double _Complex z0b = (0-0.25*I);

	double _Complex z1 = tt1/2.0;
	double _Complex z2 = cmplDiv((q1*cpow(tt1,2)),(4.*cpow(-(cpow(q1,2)*(q1 - 2*qv)*cpow(tt1,3)) + 2*csqrt(-(cpow(q1,4)*(q1 - qv)*qv*cpow(tt1,6))),0.3333333333333333)));
	double _Complex z3 = cmplDiv((z0a*csqrt(3)*q1*cpow(tt1,2)),cpow(-(cpow(q1,2)*(q1 - 2*qv)*cpow(tt1,3)) + 2*csqrt(-(cpow(q1,4)*(q1 - qv)*qv*cpow(tt1,6))),0.3333333333333333));
	double _Complex z4 = cpow(-(cpow(q1,2)*(q1 - 2*qv)*cpow(tt1,3)) + 2*csqrt(-(cpow(q1,4)*(q1 - qv)*qv*cpow(tt1,6))),0.3333333333333333)/(4.*q1);
	double _Complex z5 = (z0b*csqrt(3)*cpow(-(cpow(q1,2)*(q1 - 2*qv)*cpow(tt1,3)) + 2*csqrt(-(cpow(q1,4)*(q1 - qv)*qv*cpow(tt1,6))),0.3333333333333333))/q1;
	
	return creal(z1+z2+z3+z4+z5);
}
static double TG_POLY3_FindOnePath_txc(double q1, double tt1, double qv)
{
	double _Complex z0a = (0+0.25*I);
	double _Complex z0b = (0-0.25*I);

	double _Complex z1 = tt1/2.0;
	double _Complex z2 = cmplDiv((q1*cpow(tt1,2)),(4.*cpow(-(cpow(q1,2)*(q1 - 2*qv)*cpow(tt1,3)) + 2*csqrt(-(cpow(q1,4)*(q1 - qv)*qv*cpow(tt1,6))),0.3333333333333333)));
	double _Complex z3 = cmplDiv((z0a*csqrt(3)*q1*cpow(tt1,2)),cpow(-(cpow(q1,2)*(q1 - 2*qv)*cpow(tt1,3)) + 2*csqrt(-(cpow(q1,4)*(q1 - qv)*qv*cpow(tt1,6))),0.3333333333333333));
	double _Complex z4 = cpow(-(cpow(q1,2)*(q1 - 2*qv)*cpow(tt1,3)) + 2*csqrt(-(cpow(q1,4)*(q1 - qv)*qv*cpow(tt1,6))),0.3333333333333333)/(4.*q1);
	double _Complex z5 = (z0b*csqrt(3)*cpow(-(cpow(q1,2)*(q1 - 2*qv)*cpow(tt1,3)) + 2*csqrt(-(cpow(q1,4)*(q1 - qv)*qv*cpow(tt1,6))),0.3333333333333333))/q1;
	
	return creal(z1+z2-z3+z4-z5);
}
static bool TG_POLY3_DivideSeq(void)
{
	//Na poczatek wszystkie punkty w sekwencji maja zerowy numer subsekwenscji
	for(int num=0;num<JOINTS_MAX;num++)
	{
		for(int i=0;i<Traj.Tgen.maxpathpoints;i++)
		{
			Traj.Tgen.path[num][i].q0 = Traj.Tgen.waypoints[i].pos.v[num];
			Traj.Tgen.path[num][i].q1 = Traj.Tgen.waypoints[i+1].pos.v[num];
			Traj.Tgen.path[num][i].v0 = 0.0;
			Traj.Tgen.path[num][i].v1 = 0.0;
			Traj.Tgen.path[num][i].t1 = 0.0;
			Traj.Tgen.path[num][i].vmax = Traj.Tgen.waypoints[1].vel;
			Traj.Tgen.path[num][i].amax = accmax;
			Traj.Tgen.path[num][i].dir = 0;
			Traj.Tgen.path[num][i].n = 0;
		}
	}
	
	//Podzial na subsekwencje
	for(int num=0;num<JOINTS_MAX;num++)
	{
		for(int i=0;i<Traj.Tgen.maxpathpoints;i++)
		{
			if((Traj.Tgen.path[num][i].q1 - Traj.Tgen.path[num][i].q0) > 0.0)
			{
				Traj.Tgen.path[num][i].dir = 1;
			}
			else if((Traj.Tgen.path[num][i].q1 - Traj.Tgen.path[num][i].q0) < 0.0)
			{
				Traj.Tgen.path[num][i].dir = -1;
				Traj.Tgen.path[num][i].vmax = -fabs(Traj.Tgen.path[num][i].vmax);
			}
		}
	}
	
	for(int num=0;num<JOINTS_MAX;num++)
	{
		int number = 0;
		for(int i=1;i<Traj.Tgen.maxpathpoints;i++)
		{
			if(Traj.Tgen.path[num][i].dir == Traj.Tgen.path[num][i-1].dir)
			{
				Traj.Tgen.path[num][i].n = number;
			}
			else
			{
				number++;
				Traj.Tgen.path[num][i].n = number;
			}
		}
	}
	
	return true;
}
static bool TG_POLY3_FindOnePathOneSeq(int num, int s0, int s1)
{
	double q0 = Traj.Tgen.path[num][s0].q0;
	double q1 = Traj.Tgen.path[num][s1].q1;
	double qdist = q1 - q0;
	double vmax = Traj.Tgen.path[num][s1].vmax;
	double amax = Traj.Tgen.path[num][s1].amax;
	double t1 = 0.0;
	
	if(fabs(qdist) >= prec)
	{
		double tendFromVel = TG_POLY3_FindTendFromVel(q0, q1, 0, 0, 0, vmax, 0);
		double tendFromAcc = TG_POLY3_FindTendFromAcc(q0, q1, 0, 0, 0, 0, amax);
		if(fabs(tendFromAcc) >= fabs(tendFromVel))	t1 = tendFromAcc;
		else																				t1 = tendFromVel;
		vmax = TG_POLY3_FindMaxvelFromTend(q0, q1, 0, 0, t1, 0, amax);
		Traj.Tgen.path[num][s1].t1 = t1;
		Traj.Tgen.path[num][s1].vmax = vmax;
		
		//przypadek gdy sa punkty posrednie w subsekwencji. Wymaga to szeregu dodatkowych obliczen: w szczególnosci wymaga obliczenia chwili czasu w jakiej ma byc osiagniety dany punkt posredni w subsekwencji
		if(s1 > s0)
		{
			for(int i=s0;i<s1;i++)
			{
				double txav = TG_POLY3_FindOnePath_txa(qdist, t1, Traj.Tgen.path[num][i].q1 - Traj.Tgen.path[num][s0].q0);
				double txbv = TG_POLY3_FindOnePath_txb(qdist, t1, Traj.Tgen.path[num][i].q1 - Traj.Tgen.path[num][s0].q0);
				double txcv = TG_POLY3_FindOnePath_txc(qdist, t1, Traj.Tgen.path[num][i].q1 - Traj.Tgen.path[num][s0].q0);
				double tx = 0.0;
				if(txav > 0.0 && txav < t1)		tx = txav;
				if(txbv > 0.0 && txbv < t1)		tx = txbv;
				if(txcv > 0.0 && txcv < t1)		tx = txcv;
				Traj.Tgen.path[num][i].t1 = tx;				
			}
			
			//Wyznaczenie miedzyczasów zamiast czasu skumulowanego
			for(int i=s1;i>s0;i--)
			{
				Traj.Tgen.path[num][i].t1 = Traj.Tgen.path[num][i].t1 - Traj.Tgen.path[num][i-1].t1;
			}
		}
	}
	if(fabs(qdist) < prec)
	{
		for(int i=s0;i<=s1;i++)
		{
			Traj.Tgen.path[num][i].t1 = prec;
			Traj.Tgen.path[num][i].vmax = 0;
		}
	}
	
	return true;
}
static bool TG_POLY3_FindOnePath(int num)
{
	bool flag = false;
	int snum = 0, s0 = 0, s1 = 0;
	for(int i=0;i<Traj.Tgen.maxpathpoints;i++)
	{
		if(flag == false && Traj.Tgen.path[num][i].n == snum)
		{
			flag = true;
			s0 = i;
			s1 = i;
		}
		else if(flag == true && Traj.Tgen.path[num][i].n == snum)
		{
			s1 = i;
		}
		
		if((flag == true && (Traj.Tgen.path[num][i].n == (snum+1))) || (flag == true && i == (Traj.Tgen.maxpathpoints-1)))
		{
			TG_POLY3_FindOnePathOneSeq(num, s0, s1);
			flag = false;
			snum++;
			i--;
		}
	}
	
	return true;
}
static bool TG_POLY3_SynchronizePath(void)
{
	for(int i=0;i<Traj.Tgen.maxpathpoints;i++)
	{
		double maxval = Traj.Tgen.path[0][i].t1;
		
		for(int num=1;num<JOINTS_MAX;num++)
			if(Traj.Tgen.path[num][i].t1 > maxval)
				maxval = Traj.Tgen.path[num][i].t1;
				
		for(int num=0;num<JOINTS_MAX;num++)
			Traj.Tgen.path[num][i].t1 = maxval;
	}
	
	return true;
}
static double TG_POLY3_FindVel(double q0, double q1, double t1, double t)
{
	return (6*(q0-q1)*t*(t-t1))/pow(t1,3);
}
static bool TG_POLY3_AddVelocityOnePathOneSeq(int num, int s0, int s1)
{
	if(s0 == s1)
		return false;
	
	//Wyznaczenie czasów skumulowanych zamiast miedzyczasów
	for(int i=s0+1;i<=s1;i++)
		Traj.Tgen.path[num][i].t1 = Traj.Tgen.path[num][i].t1 + Traj.Tgen.path[num][i-1].t1;
	
	double q0 = Traj.Tgen.path[num][s0].q0;
	double q1 = Traj.Tgen.path[num][s1].q1;
	double t1 = Traj.Tgen.path[num][s1].t1;
	
	for(int i=s0;i<s1;i++)
		Traj.Tgen.path[num][i].v1 = TG_POLY3_FindVel(q0, q1, t1, Traj.Tgen.path[num][i].t1);
		
	for(int i=s0+1;i<=s1;i++)
		Traj.Tgen.path[num][i].v0 = Traj.Tgen.path[num][i-1].v1;
		
	//Wyznaczenie miedzyczasów zamiast czasu skumulowanego
	for(int i=s1;i>s0;i--)
	{
		Traj.Tgen.path[num][i].t1 = Traj.Tgen.path[num][i].t1 - Traj.Tgen.path[num][i-1].t1;
	}
	
	return true;
}
static bool TG_POLY3_AddVelocityOnePath(int num)
{
	bool flag = false;
	int snum = 0, s0 = 0, s1 = 0;
	for(int i=0;i<Traj.Tgen.maxpathpoints;i++)
	{
		if(flag == false && Traj.Tgen.path[num][i].n == snum)
		{
			flag = true;
			s0 = i;
			s1 = i;
		}
		else if(flag == true && Traj.Tgen.path[num][i].n == snum)
		{
			s1 = i;
		}
		
		if((flag == true && (Traj.Tgen.path[num][i].n == (snum+1))) || (flag == true && i == (Traj.Tgen.maxpathpoints-1)))
		{
			TG_POLY3_AddVelocityOnePathOneSeq(num, s0, s1);
			flag = false;
			snum++;
			i--;
		}
	}
	
	return true;
}
static bool TG_POLY3_FindPath(void)
{
	//Podzial waypointów na subsekwencje
	if(TG_POLY3_DivideSeq() == false)
		return false;
	
	//Wyznaczenie sciezek waypointów dla wszystkich napedów
	for(int num=0;num<JOINTS_MAX;num++)
		if(TG_POLY3_FindOnePath(num) == false)
			return false;
	
	//Synchronizacja wszystkich napedów
	if(TG_POLY3_SynchronizePath() == false)
		return false;
	
	//Wyznaczenie predkosci w punktach posrednich dla poszczególnych subsekwencji (czyli w miejscach gdzie ma byc plynne przejscie przez waypoint)
	for(int num=0;num<JOINTS_MAX;num++)
		if(TG_POLY3_AddVelocityOnePath(num) == false)
			return false;
	
	return true;
}
static bool TG_POLY3_FindTraj(void)
{
	double step = Traj.Tgen.stepTime;
	for(int num=0;num<JOINTS_MAX;num++)
	{
		int idx = 0;
		for(int i=0;i<Traj.Tgen.maxpathpoints;i++)
		{
			double q0 = Traj.Tgen.path[num][i].q0;
			double q1 = Traj.Tgen.path[num][i].q1;
			double v0 = Traj.Tgen.path[num][i].v0;
			double v1 = Traj.Tgen.path[num][i].v1;
			double t1 = Traj.Tgen.path[num][i].t1;
			
			int len = round(t1/step);
			
			for(int j=0;j<len;j++)
			{
				double t = (double)(j+1) * step;
				
				Traj.points[idx].pos[num] = TG_POLY3_q(q0,q1,v0,v1,t1,t) / pC->Joints[num].limitPosMax * MAXINT16;
				Traj.points[idx].vel[num] = TG_POLY3_dq(q0,q1,v0,v1,t1,t) / pC->Joints[num].limitVelMax * MAXINT16;
				Traj.points[idx].acc[num] = TG_POLY3_ddq(q0,q1,v0,v1,t1,t) / pC->Joints[num].limitAccMax * MAXINT16;
				idx++;
			}
		}
	}
	
	return true;
}
static double TG_POLY3_GetTrajectoryTime(void)
{
	double totalTime = 0.0;
	for(int i=0;i<Traj.Tgen.maxpathpoints;i++)
		totalTime += Traj.Tgen.path[0][i].t1;
	return totalTime;
}
void TG_TrajGen(void)
{
	if(pC->Jtc.currentFsm != JTC_FSM_HoldPos && pC->Jtc.currentFsm != JTC_FSM_Operate)
		return;
	if(Traj.Tgen.reqTrajPrepare == false)
		return;

	TG_ClearTgenVariables();
	
	if(TG_GetSeqFromMbs() == false)
	{
		Control_TrajClear();
		return;
	}
	Control_TrajClear();
	
	if(TG_POLY3_FindPath() == false)
	{
		Traj.Tgen.trajPrepStatus = TPS_UnknownError;
		Control_TrajClear();
		return;
	}
	
	//Sprawdzenie dlugosci calej trajektorii
	double totalTime = TG_POLY3_GetTrajectoryTime();
	double trajLen = totalTime / Traj.Tgen.stepTime;
	if(trajLen > TRAJ_POINTSMAX)
	{
		Traj.Tgen.trajPrepStatus = TPS_TrajToLong;
		Control_TrajClear();
		return;
	}
	
	//Generowanie punktów trajektorii
	Traj.Tgen.maxpoints = trajLen;
	if(TG_POLY3_FindTraj() == false)
	{
		Traj.Tgen.trajPrepStatus = TPS_UnknownError;
		Control_TrajClear();
		return;
	}
	
	Traj.Tgen.status = TGS_Ready;
	Traj.stepTime = 1000.0  * Traj.Tgen.stepTime; //W Tgen stepTime jest w sekundach (def 0.001), a w Traj stepTime jest w ilosci ms na punkt (def 1)
	Traj.numRecPoints = Traj.Tgen.maxpoints;
	Traj.comStatus = TCS_WasRead;
	Traj.targetTES = TES_Stop;
	Traj.Tgen.reqTrajPrepare = false;
	Traj.Tgen.trajPrepStatus = TPS_Ok;
}
