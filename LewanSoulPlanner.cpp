/*
 * LewanSoulPlanner.cpp
 *
 *  Created on: May 11, 2020
 *      Author: hephaestus
 */

#include "LewanSoulPlanner.h"

//#include <FlashStorage.h>
//Calibration Values              [-8999, 4440, 4731, -38, -10329, 8976]
int32_t startingAngles []= {-8999, 4440, 4731, -38, 10329, -8976,	0};
int32_t upperAngles []= {		9000, 	9000, 	9000, 9000, 15500, 	11000, 	11000};
int32_t lowerAngles []= {		-9000, -9000,  -9000, -13000, -6000, 	-11000, -11000};
static bool preferencesInUse=false;

//FlashStorage(cal1, float);
//FlashStorage(cal2, float);
//FlashStorage(cal3, float);
//FlashStorage(lock, int);

LewanSoulPlanner::LewanSoulPlanner(int n,int channel) {
	this->channel=channel;
	numberOfServos=n;
	int startIndex=channel==0?0:indexSplit;
	int endIndex=channel==0?indexSplit:numberOfServos;
	motors=new LX16AServo*[numberOfServos];
	for(int i=startIndex;i<endIndex;i++)
		motors[i]= new LX16AServo(&servoBus, i+1);
	servoBus.debug(true);

}

LewanSoulPlanner::~LewanSoulPlanner() {
		for(int i=0;i<numberOfServos;i++)
			delete(motors[i]);
		delete(motors);
}
bool LewanSoulPlanner::calibrate(int startIndex,int endIndex){
	for(int i=startIndex;i<endIndex;i++){
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
	read( startIndex,endIndex);
	Serial.println("\r\nStarting the motor motion after calibration");
	for(int i=startIndex;i<endIndex;i++){
		//upstream[i]->startInterpolationDegrees(startingAngles[i],2000,SIN);
		//motors[i]->move_time_and_wait_for_sync(startingAngles[i], 2000);
		targets[i]=positions[i];
	}
	//servoBus.move_sync_start();
	return true;
}
void LewanSoulPlanner::read(int startIndex,int endIndex){
	for(int i=startIndex;i<endIndex;i++){
		int32_t pos = motors[i]->pos_read();
		positions[i]=pos;
	}
}

void LewanSoulPlanner::update(int startIndex,int endIndex){
	long start = millis();
	servoBus.move_sync_start();
	read(startIndex, endIndex);
	for(int i=startIndex;i<endIndex;i++){
		int32_t target = targets[i];
		if(target>motors[i]->getMaxCentDegrees()){
			Serial.println("Index "+String(i)+" Capping upper setpoint "+String(target)+" to "+String(motors[i]->getMaxCentDegrees()));
			target=motors[i]->getMaxCentDegrees();
			targets[i]=target;
		}
		if(target<motors[i]->getMinCentDegrees()){
			Serial.println("Index "+String(i)+" Capping lower setpoint "+String(target)+" to "+String(motors[i]->getMinCentDegrees()));
			target=motors[i]->getMinCentDegrees();
			targets[i]=target;
		}
		int timingOffset = millis()-start;
		motors[i]->move_time_and_wait_for_sync(target, plannerLoopTimeMs+timingOffset+2);
	}
}
void LewanSoulPlanner::loop(){
	int startIndex=channel==0?0:indexSplit;
	int endIndex=channel==0?indexSplit:numberOfServos;
	bool clibarationRequired=false;
	switch(state){
	case StartupSerial:
		if(channel==0)
			servoBus.beginOnePinMode(&Serial1,SERIAL_BUS0); //
		else
			servoBus.beginOnePinMode(&Serial2,SERIAL_BUS1); //
		servoBus.debug(false);
		servoBus.retry = 0; // enforce synchronous real time


		Serial.println("\r\nBeginning Trajectory Planner "+String(channel));
		pinMode(HOME_SWITCH_PIN, INPUT_PULLUP);
		pinMode(MOTOR_DISABLE, INPUT_PULLUP);
		for(int i=startIndex;i<endIndex;i++)
				motors[i]->disable();
		state=waitingToreadPreferences;
		pinMode(INDICATOR, OUTPUT);
		break;
	case waitingToreadPreferences:
		if(preferencesInUse)
			break;
		preferencesInUse=true;
		delay(1);
		state=readPreferrences;
		// no break
	case readPreferrences:
		preferences.begin("Lewan", true);
		for(int i=startIndex;i<endIndex;i++){
			motors[i]->initialize();
			uint8_t key = preferences.getUChar(("key"+String(i)).c_str(), 0);
			if(key==FLASHKEY){
				Serial.println("Ch:"+String(channel)+"Accessing Stored values for "+String(i));
				motors[i]->staticOffset = preferences.getInt(("off"+String(i)).c_str(), -1);
				motors[i]->minCentDegrees= lowerAngles[i];//preferences.getInt(("min"+String(i)).c_str(), -1);
				motors[i]->maxCentDegrees= upperAngles[i];//preferences.getInt(("max"+String(i)).c_str(),-1);
			}else{
				Serial.println("Ch:"+String(channel)+" No stored values for "+String(i));
				clibarationRequired=true;
			}
		}
		preferences.end();
		preferencesInUse=false;
		if(clibarationRequired){
			state=WaitForHomePress;
		}else{
			state=running;
			read( startIndex, endIndex);
			for(int i=startIndex;i<endIndex;i++){
				targets[i]=positions[i];
				motors[i]->setLimitsTicks(	(lowerAngles[i]-motors[i]->staticOffset)/24,
											(upperAngles[i]-motors[i]->staticOffset)/24
											);
			}
			digitalWrite(INDICATOR, 1);
		}
		break;
	case waitingtoWritePreferences:
		if(preferencesInUse)
			break;
		preferencesInUse=true;
		delay(1);
		state= writePreferences;
		//no break
	case writePreferences:
		preferences.begin("Lewan", false);
		Serial.println("CH:"+String(channel)+" Storing preferences in FLASH...");
		for(int i=startIndex;i<endIndex;i++){
		   preferences.putUChar(("key"+String(i)).c_str(), FLASHKEY);
		   preferences.putInt(("off"+String(i)).c_str(),motors[i]->staticOffset);
//		   preferences.putInt(("min"+String(i)).c_str(),motors[i]->minCentDegrees);
//		   preferences.putInt(("max"+String(i)).c_str(),motors[i]->maxCentDegrees);
		}
		preferences.end();
		Serial.println("CH:"+String(channel)+" Done Storing preferences in FLASH");
		preferencesInUse=false;

		read(startIndex, endIndex);
		state=running;
		digitalWrite(INDICATOR, 1);
		break;
	case WaitForHomePress:
		read( startIndex, endIndex);

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
		read( startIndex, endIndex);
		if(millis()-timeOfLastBlink>200){
			timeOfLastBlink=millis();
			blinkState=!blinkState;
			digitalWrite(INDICATOR, blinkState?1:0);
		}
		if(millis()-timeOfHomingPressed>300 && !digitalRead(HOME_SWITCH_PIN)){// wait for motors to settle, debounce
			timeOfHomingPressed = millis();
			digitalWrite(INDICATOR, 0);
			if(calibrate( startIndex, endIndex)){
				state =WaitingForCalibrationToFinish;
			}else{
				Serial.println("\r\nCal Error");
			}
		}
		break;
	case WaitingForCalibrationToFinish:
		read(startIndex, endIndex);
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
		//servoBus.debug(true);
		state=waitingtoWritePreferences;
		digitalWrite(INDICATOR, 1);
		break;

	case WaitingToRun:
		if(millis()-timeOfLastRun==plannerLoopTimeMs){
			state=running;
			timeOfLastRun=millis();
			//Serial.print("`");
		}else if(millis()-timeOfLastRun>plannerLoopTimeMs){
			//Serial.println("\t\tERROR Real time loop broken, took: "+String((millis()-timeOfLastRun)));
			state=running;
			timeOfLastRun=millis();
		}else
			break;
		//no break
	case running:
		servoBus.debug(false);
		if(digitalRead(MOTOR_DISABLE)){
			update( startIndex, endIndex);
			state=WaitingToRun;
		}else{
			for(int i=startIndex;i<endIndex;i++)
				motors[i]->disable();
			state=disabled;
			Serial.println("\r\nDisable Motors");
		}
		break;
	case disabled:
		read(startIndex, endIndex);
//		Serial.print("\n[ ");
//		for(int i=0;i<num;i++){
//			Serial.print(String(positions[i]));
//			if(i!=num-1)
//				Serial.print(" , ");
//		}
//		Serial.print(" ] ");
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

