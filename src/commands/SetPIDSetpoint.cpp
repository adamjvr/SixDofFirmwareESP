/*
 * SetPIDSetpoint.cpp
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */

#include "SetPIDSetpoint.h"
#include <Arduino.h>
SetPIDSetpoint::SetPIDSetpoint(int num, LewanSoulPlanner * planner) :
		PacketEventAbstract(1848) {
	numPID = num;
	this->planner = planner;

}

void SetPIDSetpoint::event(float * buffer) {
	for (int i = 0; i < numPID; i++) {
		planner->positions[i]=buffer[i+2];
	}

}
