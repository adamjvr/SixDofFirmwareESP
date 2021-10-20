/*
 * LewanSoulPlanner.h
 *
 *  Created on: May 11, 2020
 *      Author: hephaestus
 */

#ifndef SRC_PID_LEWANSOULPLANNER_H_
#define SRC_PID_LEWANSOULPLANNER_H_

#include <lx16a-servo.h>
#define HOME_SWITCH_PIN 0
#define INDICATOR 13
#define MOTOR_DISABLE 12
#define plannerLoopTimeMs 15
enum LewanSoulState_t {
	StartupSerial, WaitForHomePress,WaitForHomeRelease,WaitingForCalibrationToFinish,WaitingToRun,running,disabled
// Add more states here and be sure to add them to the cycle
};
class LewanSoulPlanner {
	LX16ABus servoBus;
	LX16AServo ** motors;
	int num=0;
	LewanSoulState_t state=StartupSerial;
	long timeOfLastRun = 0;
	long timeOfHomingPressed=0;
	bool blinkState = false;
	long timeOfLastBlink = 0;
public:
	LewanSoulPlanner( int num);
	~LewanSoulPlanner();
	void loop();
	void update();
	void read();
	bool calibrate();
	int targets[10]={0,};
	int positions[10]={0,};
};

#endif /* SRC_PID_LEWANSOULPLANNER_H_ */
