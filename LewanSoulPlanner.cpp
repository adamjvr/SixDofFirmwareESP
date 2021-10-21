/*
 * LewanSoulPlanner.cpp
 *
 *  Created on: May 11, 2020
 *      Author: hephaestus
 */

#include "LewanSoulPlanner.h"

//#include <FlashStorage.h>
int32_t startingAngles []= {-8999, 4440, 4731, 8962, 3171, 9024,0};
int32_t upperAngles []= {9000, 9500, 9000, 11000, 19500, 11000, 11000};
int32_t lowerAngles []= {-9000, -8500, -9000, -11000, -2500, -11000, -11000};

//FlashStorage(cal1, float);
//FlashStorage(cal2, float);
//FlashStorage(cal3, float);
//FlashStorage(lock, int);

LewanSoulPlanner::LewanSoulPlanner(int n) {
	num=n;
	motors=new LX16AServo*[num];
	for(int i=0;i<num;i++)
		motors[i]= new LX16AServo(&servoBus, i+1);

}

LewanSoulPlanner::~LewanSoulPlanner() {
		for(int i=0;i<num;i++)
			delete(motors[i]);
		delete(motors);
}
bool LewanSoulPlanner::calibrate(){
	for(int i=0;i<num;i++){
		Serial.println("Attempt Calibrating "+String(motors[i]->_id)+"...");
		if(!motors[i]->calibrate(startingAngles[i],lowerAngles[i],upperAngles[i])){
			return false;
		}
		int32_t pos = startingAngles[i]-motors[i]->pos_read();
		if(abs(pos)>24){
			Serial.println("Settling Error of"+String(pos)+", re-calibrating on index "+String(motors[i]->_id));
			return false;
		}
		Serial.println("Done Calibrating "+String(motors[i]->_id)+" OK!\r\n");
	}
	delay(1000);
	read();
	Serial.println("\r\nStarting the motor motion after calibration");
	for(int i=0;i<num;i++){
		//upstream[i]->startInterpolationDegrees(startingAngles[i],2000,SIN);
		//motors[i]->move_time_and_wait_for_sync(startingAngles[i], 2000);
		targets[i]=positions[i];
	}
	//servoBus.move_sync_start();
	return true;
}
void LewanSoulPlanner::read(){
	for(int i=0;i<num;i++){
		int32_t pos = motors[i]->pos_read();
		positions[i]=pos;
	}
}

void LewanSoulPlanner::update(){
	long start = millis();
	servoBus.move_sync_start();
	read();
	for(int i=0;i<num;i++){
		int32_t target = targets[i];
		if(target>motors[i]->getMaxCentDegrees()){
			Serial.println("Capping upper setpoint "+String(target)+" to "+String(motors[i]->getMaxCentDegrees()));
			target=motors[i]->getMaxCentDegrees();
			targets[i]=target;
			targets[i]=target;
		}
		if(target<motors[i]->getMinCentDegrees()){
			Serial.println("Capping lower setpoint "+String(target)+" to "+String(motors[i]->getMinCentDegrees()));
			target=motors[i]->getMinCentDegrees();
			targets[i]=target;
		}
		int timingOffset = millis()-start;
		motors[i]->move_time_and_wait_for_sync(target, plannerLoopTimeMs+timingOffset+2);
	}
}
void LewanSoulPlanner::loop(){

	switch(state){
	case StartupSerial:

		servoBus.beginOnePinMode(&Serial1,14); // use pin 2 as the TX flag for buffer
		servoBus.debug(true);
		servoBus.retry = 0; // enforce synchronous real time
		//servoBus.debug(true);
		Serial.println("\r\nBeginning Trajectory Planner");
		pinMode(HOME_SWITCH_PIN, INPUT_PULLUP);
		pinMode(MOTOR_DISABLE, INPUT_PULLUP);
		for(int i=0;i<num;i++)
				motors[i]->disable();
		state=WaitForHomePress;
		pinMode(INDICATOR, OUTPUT);
		break;
	case WaitForHomePress:
		read();

		if(!digitalRead(HOME_SWITCH_PIN)){
			timeOfHomingPressed = millis();
			state = WaitForHomeRelease;
			Serial.println("\r\nUSER Button PRESSED!");
			digitalWrite(INDICATOR, 0);
		}else{
			if(millis()-timeOfLastBlink>1000){
				timeOfLastBlink=millis();
				blinkState=!blinkState;
				digitalWrite(INDICATOR, blinkState?1:0);
				//Serial.println("Waiting for Home...");
			}
		}
		break;
	case WaitForHomeRelease:
		read();
		if(millis()-timeOfLastBlink>200){
			timeOfLastBlink=millis();
			blinkState=!blinkState;
			digitalWrite(INDICATOR, blinkState?1:0);
		}
		if(millis()-timeOfHomingPressed>300 && !digitalRead(HOME_SWITCH_PIN)){// wait for motors to settle, debounce
			timeOfHomingPressed = millis();
			digitalWrite(INDICATOR, 0);
			if(calibrate()){
				state =WaitingForCalibrationToFinish;
			}else{
				Serial.println("\r\nCal Error");
			}
		}
		break;
	case WaitingForCalibrationToFinish:
		read();
		if(millis()-timeOfLastBlink>50){
			timeOfLastBlink=millis();
			blinkState=!blinkState;
			digitalWrite(INDICATOR, blinkState?1:0);
		}
		if(!digitalRead(HOME_SWITCH_PIN)){
			//still on the homing switch
			break;
		}
//		for(int i=0;i<num;i++){
//			if(!upstream[i]->isInterpolationDone())
//				break;// not done yet
//		}
		Serial.println("\r\nStarting the planner");
		state=running;
		digitalWrite(INDICATOR, 1);
		break;
	case WaitingToRun:
		if(millis()-timeOfLastRun>plannerLoopTimeMs){
			state=running;
			timeOfLastRun=millis();
		}
		break;
	case running:
		if(digitalRead(MOTOR_DISABLE)){
			update();
			state=WaitingToRun;
		}else{
			for(int i=0;i<num;i++)
				motors[i]->disable();
			state=disabled;
			Serial.println("\r\nDisable Motors");
		}
		break;
	case disabled:
		read();
		Serial.print("\n[ ");
		for(int i=0;i<num;i++){
			Serial.print(String(positions[i]));
			if(i!=num-1)
				Serial.print(" , ");
		}
		Serial.print(" ] ");
		if(digitalRead(MOTOR_DISABLE)){
			state=running;
			Serial.println("\r\nEnable Motors");
		}else
			if(!digitalRead(HOME_SWITCH_PIN)){
				state=WaitForHomePress;
			}
		break;
	}
}

