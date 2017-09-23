/* File PmodHB3.c
@Author :- Nishad Saraf, Chaitanya Deshpande

*
* This file implements the driver functions for the H-bridge
* custom peripheral.  It consists of drivers through which we can
* read and write values into registers of custom IP.
*

*/

/***************************** Include Files *******************************/
#include "PmodHB3.h"

/************************** Function Definitions ***************************/

/* Initialize the H-bridge peripheral driver
*
* Saves the Base address of the  H-bridge peripheral and runs the selftest
*
* @param	BaseAddr is the base address of the H-bridge register set
*
* @return
* 		- XST_SUCCESS	Initialization was successful.
*
* @note		This function can hang if the peripheral was not created correctly
* @note		The Base Address of the H-bridge peripheral will be in xparameters.h

uint32_t PMODHB3_initialize(p_pmodHB3 p_instance, uint32_t baseaddr)
{
	// Save the Base Address of the PmodHB3 register set so we know where to point the driver
	p_instance->Base_address = baseaddr;

	// Run the driver self-test.
	if ( XST_SUCCESS == PMODHB3_SelfTest(p_instance->Base_address ) )
	{
		p_instance->Is_ready = true;
	}
	else
	{
		p_instance->Is_ready = false;
	}

	return (p_instance->Is_ready) ? XST_SUCCESS : XST_FAILURE;
}

/*

Setting duty cycle of the PWM wave and writing State of PWM wave into
PmodHB3 register ( 1 or 0 )

*/

void PMODHB3_setDutyCycle(p_pmodHB3 p_instance, u8 dutyCycle)
{
	PMODHB3_mWriteReg(p_instance->Base_address,PMODHB3_DUTY_OFFSET,dutyCycle);
}

/*

Reading value of duty cycle of the PWM wave by reading value from Pmod H-bridge 
register ( 1 or 0 )

*/

u8 PMODHB3_readDutyCycle(p_pmodHB3 p_instance)
{
	return PMODHB3_mReadReg(p_instance->Base_address,PMODHB3_DUTY_OFFSET);
}

/*

Reading value of highcount or reading frequency of signal generated at the output
of hall sensor which is input to PmodHB3.

*/


uint32_t PMODHB3_read_SA_Highcount(p_pmodHB3 p_instance, uint32_t *p_sa_highcount)
{
	uint32_t SA_HighCount;

	SA_HighCount = PMODHB3_mReadReg(p_instance->Base_address, PMODHB3_SA_HIGH_OFFSET);
	if (SA_HighCount != p_instance->SA_HighCount)
	{
		p_instance->SA_HighCount = SA_HighCount;
	}
	*p_sa_highcount = p_instance->SA_HighCount;

	return XST_SUCCESS;
}

/* 
Setting direction of the motor by storing 1 or 0 value in Pmod HB3 register
1 for clockwise and 0 for anticlockwise

*/

void PMODHB3_setDirection(p_pmodHB3 p_instance,bool isAntiClockwise)
{
	u32 data;
	if(isAntiClockwise) data = 0;
	else data = 1;
	PMODHB3_mWriteReg(p_instance->Base_address,PMODHB3_DIR_OFFSET,data);
}

/* 
Reading direction of the motor by reading value from Pmod HB3 register
1 for clockwise and 0 for anticlockwise

*/

bool PMODHB3_readDirection(p_pmodHB3 p_instance)
{
	u32 data;
	data = PMODHB3_mReadReg(p_instance->Base_address,PMODHB3_DIR_OFFSET);
	if(data) return true;
	else return false;
}
