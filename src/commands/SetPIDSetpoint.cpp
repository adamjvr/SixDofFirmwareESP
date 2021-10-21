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
//	Serial.print("\nGot setpoint command [");
	for (int i = 0; i < numPID; i++) {
		planner->targets[i]=buffer[i+2];
//		Serial.print(String(planner->targets[i]));
//		Serial.print(String(" , "));
	}
//	Serial.print(String("]"));

}
