/*
 * LaserRange.h
 *
 *  Created on: Oct 11, 2014
 *      Author: Oomkes
 */
#include "WPILib.h"

/*
 * Description:
 */
class LaserRange{
	I2C VL6180;
	Task m_task; 
	
public:
	LaserRange();
	~LaserRange();
	int GetRange(void);
	void Init(void);

};
