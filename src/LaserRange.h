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
	int m_distance;

	
public:
	LaserRange();
	~LaserRange();
	int GetDistance(void);
	void Init(void);
	void GetRange(void);

};
