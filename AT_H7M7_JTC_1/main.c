#include "Control.h"
int main(void)
{
 	Control_SystemConf();
	while(1)
	{
		Control_JtcJogKinCalc();
		TG_TrajGen();
	}
}
