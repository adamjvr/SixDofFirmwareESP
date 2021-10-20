/*
 * GetPIDData.h
 *
 *  Created on: Nov 9, 2018
 *      Author: hephaestus
 */

#ifndef SRC_COMS_GETPIDDATA_H_
#define SRC_COMS_GETPIDDATA_H_
#include <SimplePacketComs.h>
#include "../../LewanSoulPlanner.h"
class GetPIDData: public PacketEventAbstract {
private:
	LewanSoulPlanner * planner;
	int numPID;
public:
	GetPIDData(int num, LewanSoulPlanner * planner);
	virtual ~GetPIDData();
	void event(float * buffer);
};

#endif /* SRC_COMS_GETPIDDATA_H_ */
