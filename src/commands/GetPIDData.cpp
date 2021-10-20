/*
 * GetPIDData.cpp
 *
 *  Created on: Nov 9, 2018
 *      Author: hephaestus
 */

#include "GetPIDData.h"

GetPIDData::~GetPIDData() {
	// TODO Auto-generated destructor stub
}

GetPIDData::GetPIDData(int num, LewanSoulPlanner * planner) :
		PacketEventAbstract(1910) {
	numPID = num;
	this->planner = planner;

}

void GetPIDData::event(float * buffer) {
	buffer[0] = (float) numPID;
	for (int i = 0; i < numPID && i < 7; i++) {
		buffer[i * 2 + 1 + 0] = planner->targets[i];
		buffer[i * 2 + 1 + 1] = planner->positions[i];
	}

}

