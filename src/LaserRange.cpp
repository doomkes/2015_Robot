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
 		lr->GetRange();
 		//distance = distance +.01;
 		//SmartDashboard::PutNumber("Laser Range", lr->m_distance);
		Wait(.05); // hog prevention
}
}

void LaserRange::GetRange()
{
	uint8_t bytes_out[3] = {0x0,0x4f,0x3}; // Index = 0x04f, data = 0x3 // interrupt status
	uint8_t byte_in;
	//VL6180.Transaction(bytes_out,2,&byte_in,1);
	//SmartDashboard::PutNumber("Laser interrupt status", byte_in);
	uint8_t clawDistance = 99;

	bool range_error = false;
	//if(byte_in & 0x4) // new range sample ready
	if (true)
	{
		// check for range error
		//bytes_out[0] = 0;
		//bytes_out[1] = 0x4d;
		//VL6180.Transaction(bytes_out,2,&byte_in,1);
		//SmartDashboard::PutNumber("Laser range error", byte_in);
		//if(byte_in & 0xf0) range_error = true; // upper 4 bits contain error codes

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
	bytes_out[3] = 7; // clear range flag
	VL6180.WriteBulk(bytes_out,3);
	m_distance = clawDistance;
	//SmartDashboard::PutNumber("Laser Range", m_distance);
	return;// clawDistance;
}




LaserRange::LaserRange(): // Constructor
		    VL6180(I2C::kOnboard,0x29), // 7 bit address
			m_task ("LaserSensor", (FUNCPTR)ReadSensor),
			m_distance(255)
{

}

LaserRange::~LaserRange() // Destructor
{	

}

int LaserRange::GetDistance() // Getter
{
	return m_distance;
}

void LaserRange::Init(void){

	// CLAW RANGE INIT
	// set measurement period  to 30 mS
	uint8_t bytes_out[3] = {0x0,0x1b,0x3}; // Index = 0x01b, data = 0x2
	uint8_t byte_in;

	bytes_out[0] = 0x2;
	bytes_out[1] = 0x07;
	bytes_out[2] = 0x1;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x2;
	bytes_out[1] = 0x08;
	bytes_out[2] = 0x1;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x1;
	bytes_out[1] = 033;
	bytes_out[2] = 0x1;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0x96;
	bytes_out[2] = 0x0;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0x97;
	bytes_out[2] = 0xfd;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0e3;
	bytes_out[2] = 0x0;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0xe4;
	bytes_out[2] = 0x4;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0xe5;
	bytes_out[2] = 0x2;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0e6;
	bytes_out[2] = 0x1;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0xe7;
	bytes_out[2] = 0x3;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0xf5;
	bytes_out[2] = 0x2;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x00;
	bytes_out[1] = 0xd9;
	bytes_out[2] = 0x5;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0xdb;
	bytes_out[2] = 0xce;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0xdc;
	bytes_out[2] = 0x3;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0xdd;
	bytes_out[2] = 0xf8;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0x9f;
	bytes_out[2] = 0x0;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0xa3;
	bytes_out[2] = 0x3c;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0xb7;
	bytes_out[2] = 0x0;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0xbb;
	bytes_out[2] = 0x3c;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0xb2;
	bytes_out[2] = 0x09;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0xca;
	bytes_out[2] = 0x9;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x01;
	bytes_out[1] = 0x98;
	bytes_out[2] = 0x1;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x1; // top 2nd col
	bytes_out[1] = 0xb0;
	bytes_out[2] = 0x17;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x1;
	bytes_out[1] = 0xad;
	bytes_out[2] = 0x0;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0xff;
	bytes_out[2] = 0x5;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x01;
	bytes_out[1] = 0x0;
	bytes_out[2] = 0x5;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x1;
	bytes_out[1] = 0x99;
	bytes_out[2] = 0x5;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x01;
	bytes_out[1] = 0x09;
	bytes_out[2] = 0x07;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x01;
	bytes_out[1] = 0x0a;
	bytes_out[2] = 0x30;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x00;
	bytes_out[1] = 0x3f;
	bytes_out[2] = 0x46;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x01;
	bytes_out[1] = 0xa6;
	bytes_out[2] = 0x1b;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x01;
	bytes_out[1] = 0xac;
	bytes_out[2] = 0x3e;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x01;
	bytes_out[1] = 0xa7;
	bytes_out[2] = 0x1f;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x01;
	bytes_out[1] = 0x03;
	bytes_out[2] = 0x01;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x00;
	bytes_out[1] = 0x30;
	bytes_out[2] = 0x00;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x00;
	bytes_out[1] = 0x1b;
	bytes_out[2] = 0x0a;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x00;
	bytes_out[1] = 0x3e;
	bytes_out[2] = 0x0a;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x01;
	bytes_out[1] = 0x31;
	bytes_out[2] = 0x04;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x00;
	bytes_out[1] = 0x11;
	bytes_out[2] = 0x10;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x00;
	bytes_out[1] = 0x14;
	bytes_out[2] = 0x24;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x00;
	bytes_out[1] = 0x31;
	bytes_out[2] = 0xff;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x00;
	bytes_out[1] = 0xd2;
	bytes_out[2] = 0x01;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x00;
	bytes_out[1] = 0xf2;
	bytes_out[2] = 0x01;
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	// Start Range measurements
	bytes_out[0] = 0x0;
	bytes_out[1] = 0x18;
	bytes_out[2] = 0x0; // Index = 0x018, data = 0x3
	VL6180.Transaction(bytes_out,3,&byte_in, 0);

	bytes_out[0] = 0x0;
	bytes_out[1] = 0x18;
	bytes_out[2] = 0x3; // Index = 0x018, data = 0x3
	VL6180.Transaction(bytes_out,3,&byte_in, 0);
	m_task.Start((int32_t)this);
	
}






