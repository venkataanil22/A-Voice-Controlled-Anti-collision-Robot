/** @file IPHB3.c
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

/***************************** Include Files *******************************/
#include "IPhb3.h"

/************************** Constant Definitions *****************************/

// bit masks for the pmodENC peripheral

// status register
#define IPhb3_direction_mask 	0x00000001
#define IPhb3_motor_speed_mask	0x000000FF
#define IPhb3_enable_mask		0x00000100
#define IPhb3_SB_rpm_mask		0xFFFF0000
#define IPhb3_SA_rpm_mask		0x0000FFFF
#define IPhb3_SB_mask			0x00000001
#define IPhb3_SA_mask			0x00000100


/************************** Function Definitions ***************************/

/****************************************************************************/
/**
* @brief write the PWM for motor speed and set motor enable
*
* writes the 0-255 value to be read by MPWM to drive motor 
*
* @param	p_instance is a pointer to the IPHB3 driver instance 
* @param	speed is a pointer to where the HW reads the PWM value
*
* @return	XST_SUCCESS
*****************************************************************************/
void write_motor_speed(p_IPhb3 instance)
{
	uint32_t Data;
	uint8_t speed;
	unit32_t motor_speed_32bit;

	Data = IPhb3_motor_speed_mask;
	speed = instance->motor_speed;
	motor_speed_32bit = speed | Data;

	IPHB3_mWriteReg(instance->base_addr, 0, motor_speed_32bit);
	return XST_SUCCESS;
}


/****************************************************************************/
/**
* @brief write to the direction pin
*
* Should only be able to write from splash screen, back up protection in
* the hardware to only change direction if SA_RPM is zero and motor enable off
*
* @param	p_instance is a pointer to the IPHB3 driver instance 
* @param	dir is a pointer to where the direction is read
*
* @return	XST_SUCCESS
*****************************************************************************/
void write_direction(p_IPhb3 instance)
{
	uint32_t Data0;
	uint8_t dir;
	uint32_t direction_32bit;

	Data0 = IPhb3_direction_mask;
	dir = instance->direction;
	direction32_bit = dir | Data3;

	IPHB3_mWriteReg(instance->base_addr, 0, direction_32bit);
	return XST_SUCCESS;
}

/****************************************************************************/
/**
* @brief read and return the value of the feedback line in 
*
* Returns the value of the SA line.  This is a single bit in a feild of 8
* Updates the SA if changed 
*
* @param	p_instance is a pointer to the pmodENC driver instance 
* @param	SA is a pointer to where the value of the signal is stored
*
* @return	XST_SUCCESS
*****************************************************************************/
void get_SA(p_IPhb3 p_instance, int8_t* SA)
{
	uint32_t read_sa;
	read_sa = IPHB3_mReadReg(p_instance->base_addr, 2);
	if(read_sa != p_instance->SA)
	{
		instance->SA = (uint8_t)read_sa; 
	}
	*SA = instance->read_sa;
	return XST_SUCCESS;
}


/****************************************************************************/
/**
* @brief read and return the 16 bit rpm value of hall sensor from SA
*
* Returns the motor rpm.  The rpm is a 16-bit unsigned integer 
* Updates the count in the IPHB3 instance if the count has changed
*
* @param	p_instance is a pointer to the IPHB3 driver instance 
* @param	SA_rpm is a pointer to where the rpm is returned
*
* @return	XST_SUCCESS
*****************************************************************************/
void get_rpm(p_IPhb3 instance, uint16_t* SA_rpm)
{
	uint32_t read_rpm;
	read_rpm = IPHB3_mReadReg(p_instance->base_addr,2);
	if(read_rpm != p_instance->SA_rpm)
	{
		instance->SA = (uint16_t)read_rpm; 
	}
	*SA_rpm = instance->read_rpm;
	return XST_SUCCESS;
}	
