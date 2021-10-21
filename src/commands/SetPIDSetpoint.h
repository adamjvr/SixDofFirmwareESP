/*
 * SetPIDSetpoint.h
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */

#ifndef SRC_COMMANDS_SETPIDSETPOINT_H_
#define SRC_COMMANDS_SETPIDSETPOINT_H_
#include <SimplePacketComs.h>
#include "../../LewanSoulPlanner.h"

class SetPIDSetpoint: public PacketEventAbstract {
	LewanSoulPlanner * planner;LewanSoulPlanner * planner2;
	int numPID;
public:
	SetPIDSetpoint(int num, LewanSoulPlanner * planner1, LewanSoulPlanner * planner2);
	virtual ~SetPIDSetpoint(){}
	void event(float * buffer);
};

#endif /* SRC_COMMANDS_SETPIDSETPOINT_H_ */
