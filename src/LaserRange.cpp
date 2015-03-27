/*
 * LaserRange.cpp
 *
 *  Created on: Oct 11, 2014
 *      Author: Oomkes
 */

#include <wpilib.h>
#include "LaserRange.h"


static void ReadSensor(LaserRange *lr)
{
while(1){
 		int startTime =GetFPGATime();
 		static double distance = 0;
 		//distance = lr->GetRange();
 		distance = distance +.01;
 		SmartDashboard::PutNumber("Laser Range", distance);
		Wait(.5); // hog prevention
}
}

int LaserRange::GetRange()
{
	uint8_t bytes_out[3] = {0x0,0x4f,0x3}; // Index = 0x04f, data = 0x3 // interrupt status
	uint8_t byte_in;
	VL6180.Transaction(bytes_out,3,&byte_in,1);
	uint8_t clawDistance = -1;
	bool range_error;
	if(byte_in & 0x4) // new range sample ready
	{
		// check for range error
		bytes_out[0] = 0;
		bytes_out[1] = 0x4d;
		VL6180.Transaction(bytes_out,2,&byte_in,1);
		if(byte_in & 0xf0) range_error = true; // upper 4 bits contain error codes

		if (!range_error) // get value
			{
			bytes_out[0] = 0;
			bytes_out[1] = 0x62;
			VL6180.Transaction(bytes_out,2,&byte_in,1);
			clawDistance = byte_in;
			}
		else clawDistance = -2;
	}
	// clear range ready flag
	bytes_out[0] = 0;
	bytes_out[1] = 0x15; // system interrupt clear
	bytes_out[3] = 1; // clear range flag
	VL6180.WriteBulk(bytes_out,3);
	return clawDistance;
}




LaserRange::LaserRange(): // Constructor
		    VL6180(I2C::kOnboard,0x52),
			m_task ("LaserSensor", (FUNCPTR)ReadSensor)
{

}

LaserRange::~LaserRange() // Destructor
{	
}


void LaserRange::Init(void){

	// CLAW RANGE INIT
	// set measurement period  to 30 mS
	uint8_t bytes_out[3] = {0x0,0x1b,0x2}; // Index = 0x01b, data = 0x2
	VL6180.WriteBulk(bytes_out,3);
	// Start Range measurements

	bytes_out[0] = 0x0;
	bytes_out[1] = 0x18;
	bytes_out[2] = 0x3; // Index = 0x018, data = 0x3
	VL6180.WriteBulk(bytes_out,3);

	m_task.Start((int32_t)this);
	
}






