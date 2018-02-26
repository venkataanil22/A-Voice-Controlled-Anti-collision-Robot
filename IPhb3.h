/** @file IPHB3.h
*
* @author Aditya Pawar (adipawar@pdx.edu)
* @copyright Portland State University, 2017
*
* @brief
* This header file contains the constants and low level function for the HB3 custom AXI Slave
* peripheral driver.  The peripheral provides access to a Digilent pmod HB3.  The HB3 contains a direction pin 
* an enable pin which is driven with PWM and two feedback lines SA and SB
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -----------------------------------------------
* 1.00a	AP	21-Feb-2017	First release of driver
* </pre>
*
******************************************************************************/
#ifndef IPHB3_H
#define IPHB3_H


/****************** Include Files ********************/
#include "stdint.h"
#include "stdbool.h"
#include "xil_types.h"
#include "xil_io.h"
#include "xstatus.h"

/************* Constant Dclarations *****************/
// register declarations
#define IPHB3_S00_AXI_SLV_REG0_OFFSET 0
#define IPHB3_S00_AXI_SLV_REG1_OFFSET 4
#define IPHB3_S00_AXI_SLV_REG2_OFFSET 8
#define IPHB3_S00_AXI_SLV_REG3_OFFSET 12


/**************************** Type Definitions *****************************/
typedef struct
{
	uint32_t	base_addr;				// base address of instance
	unit8_t 	direction;				// rotation of motor direction
	uint8_t 	enable;					// PWM pin to control speed
	uint8_t 	motor_speed;			// PWM 0-255 	
	uint16_t 	SB_rpm;					// 16 bit freq of SB feedback
	uint16_t 	SA_rpm;					// 16 bit freq of SA feedback
	unit8_t 	SB;						// feedback signal
	unit8_t 	SA;						// feedback signal
	bool	 	is_ready;
}IPhb3, *p_iphb3;

/**
 *
 * Write a value to a IPHB3 register. A 32 bit write is performed.
 * If the component is implemented in a smaller width, only the least
 * significant data is written.
 *
 * @param   BaseAddress is the base address of the IPHB3device.
 * @param   RegOffset is the register offset from the base to write to.
 * @param   Data is the data written to the register.
 *
 * @return  None.
 *
 * @note
 * C-style signature:
 * 	void IPHB3_mWriteReg(u32 BaseAddress, unsigned RegOffset, u32 Data)
 *
 */

#define IPHB3_mWriteReg(BaseAddress, RegOffset, Data) \
  	Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))

/**
 *
 * Read a value from a IPHB3 register. A 32 bit read is performed.
 * If the component is implemented in a smaller width, only the least
 * significant data is read from the register. The most significant data
 * will be read as 0.
 *
 * @param   BaseAddress is the base address of the IPHB3 device.
 * @param   RegOffset is the register offset from the base to write to.
 *
 * @return  Data is the data from the register.
 *
 * @note
 * C-style signature:
 * 	u32 IPHB3_mReadReg(u32 BaseAddress, unsigned RegOffset)
 *
 */
#define IPHB3_mReadReg(BaseAddress, RegOffset) \
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
 * @param   baseaddr_p is the base address of the IPHB3 instance to be worked on.
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
uint32_t IPHB3_initialize(p_IPhb3 instance, uint32_t baseaddr);
XStatus IPHB3_Reg_SelfTest(void * baseaddr_p);

/************************** Function Prototypes ****************************/


//HB3 functions
void write_motor_speed(p_IPhb3 instance);
void write_direction(p_IPhb3 instance);
void get_rpm(p_IPhb3 p_instance, unint16_t* SA_rpm);
void get_SA(p_IPhb3 p_instance, int8_t* SA);
void shut_down(p_IPhb3 instance);

#endif // IPHB3_H
