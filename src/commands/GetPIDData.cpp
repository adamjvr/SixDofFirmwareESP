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

GetPIDData::GetPIDData(int num, LewanSoulPlanner * planner1, LewanSoulPlanner * planner2) :
		PacketEventAbstract(1910) {
	numPID = num;
	this->planner = planner1;
	this->planner2=planner2;
}

void GetPIDData::event(float * buffer) {
	buffer[0] = (float) numPID;
	for (int i = 0; i < numPID && i < 7; i++) {
		LewanSoulPlanner * p =i<planner->indexSplit?planner:planner2;

		buffer[i * 2 + 1 + 0] = p->targets[i];
		buffer[i * 2 + 1 + 1] = p->positions[i];
	}

}

