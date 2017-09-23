/** @file PmodHB3.h
*
* @author Nishad Saraf, Chaitanya Deshpande
* 
*
* @brief
* This header file contains the constants and low level function for the PmodHB3 custom AXI Slave
* peripheral driver.  The peripheral provides access to a Digilent PmodHB3.
*
*/
#ifndef PMODHB3_H
#define PMODHB3_H


/****************** Include Files ********************/
#include "xil_types.h"
#include "xstatus.h"
#include "stdbool.h"

#define PMODHB3_S00_AXI_SLV_REG0_OFFSET 0
#define PMODHB3_S00_AXI_SLV_REG1_OFFSET 4
#define PMODHB3_S00_AXI_SLV_REG2_OFFSET 8
#define PMODHB3_S00_AXI_SLV_REG3_OFFSET 12

#define PMODHB3_DIR_OFFSET 		PMODHB3_S00_AXI_SLV_REG0_OFFSET
#define PMODHB3_DUTY_OFFSET 	PMODHB3_S00_AXI_SLV_REG1_OFFSET
#define PMODHB3_SA_HIGH_OFFSET 	PMODHB3_S00_AXI_SLV_REG2_OFFSET
#define PMODHB3_SA_LOW_OFFSET 	PMODHB3_S00_AXI_SLV_REG3_OFFSET

/**************************** Type Definitions *****************************/
typedef struct
{
	uint32_t	Base_address;		// base address for PmodHB3 peripheral registers
	bool		Is_ready;			// PmodHB3 driver has been successfully initialized
	uint32_t	SA_HighCount;				// current SA high count
	
} PmodHB3, *p_pmodHB3;

/**
 *
 * Write a value to a PMODHB3 register. A 32 bit write is performed.
 * If the component is implemented in a smaller width, only the least
 * significant data is written.
 *
 * @param   BaseAddress is the base address of the PMODHB3device.
 * @param   RegOffset is the register offset from the base to write to.
 * @param   Data is the data written to the register.
 *
 * @return  None.
 *
 * @note
 * C-style signature:
 * 	void PMODHB3_mWriteReg(u32 BaseAddress, unsigned RegOffset, u32 Data)
 *
 */
#define PMODHB3_mWriteReg(BaseAddress, RegOffset, Data) \
  	Xil_Out32((BaseAddress) + (RegOffset), (u8)(Data))

/**
 *
 * Read a value from a PMODHB3 register. A 32 bit read is performed.
 * If the component is implemented in a smaller width, only the least
 * significant data is read from the register. The most significant data
 * will be read as 0.
 *
 * @param   BaseAddress is the base address of the PMODHB3 device.
 * @param   RegOffset is the register offset from the base to write to.
 *
 * @return  Data is the data from the register.
 *
 * @note
 * C-style signature:
 * 	u32 PMODHB3_mReadReg(u32 BaseAddress, unsigned RegOffset)
 *
 */
#define PMODHB3_mReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))

/************************** Function Prototypes ****************************/
/**
 *
 * Run a self-test on the driver/device. Note this may be a destructive test if
 * resets of the device are performed.
 *
 * If the hardware system is not built correctly, this function may never
 * return to the caller.
 *
 * @param   baseaddr_p is the base address of the PMODHB3 instance to be worked on.
 *
 * @return
 *
 *    - XST_SUCCESS   if all self-test code passed
 *    - XST_FAILURE   if any self-test code failed
 *
 * @note    Caching must be turned off for this function to work.
 * @note    Self test may fail if data memory and device are not on the same bus.
 *
 */
 
 // self test and initialization functions
uint32_t PMODHB3_SelfTest(uint32_t baseaddr);
uint32_t PMODHB3_initialize(p_pmodHB3 p_instance, uint32_t baseaddr);

// PmodHB3 Functions
// Set and Read dutyCycle
void PMODHB3_setDutyCycle(p_pmodHB3 p_instance, u8 dutyCycle);
u8 PMODHB3_readDutyCycle(p_pmodHB3 p_instance);

// Frequency measurement of Hall sensor output
uint32_t PMODHB3_read_SA_Highcount(p_pmodHB3 p_instance, uint32_t* p_sa_highcount);

// Set and read direction of motor
void PMODHB3_setDirection(p_pmodHB3 p_instance,bool isClockwise);
bool PMODHB3_readDirection(p_pmodHB3 p_instance);

#endif // PMODHB3_H
