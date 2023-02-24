/*
 * LewanSoulPlanner.h
 *
 *  Created on: May 11, 2020
 *      Author: hephaestus
 */

#ifndef SRC_PID_LEWANSOULPLANNER_H_
#define SRC_PID_LEWANSOULPLANNER_H_

#include <lx16a-servo.h>
#include <Preferences.h>
#define HOME_SWITCH_PIN 0
#define INDICATOR 13
#define MOTOR_DISABLE 12
#define plannerLoopTimeMs 30
#define FLASHKEY 37
#define SERIAL_BUS0 14
#define SERIAL_BUS1 27
enum LewanSoulState_t {
	StartupSerial, WaitForHomePress,WaitForHomeRelease,WaitingForCalibrationToFinish,WaitingToRun,running,disabled,waitingToreadPreferences,readPreferrences,waitingtoWritePreferences,writePreferences
// Add more states here and be sure to add them to the cycle
};
class LewanSoulPlanner {
	Preferences preferences;
	LX16ABus servoBus;
	LX16AServo ** motors;
	int numberOfServos=0;
	long timeOfLastRun = 0;
	long timeOfHomingPressed=0;
	bool blinkState = false;
	long timeOfLastBlink = 0;
	void update(int startIndex,int endIndex);
	void read(int startIndex,int endIndex);
	bool calibrate(int startIndex,int endIndex);
	int channel=0;
public:
	LewanSoulPlanner( int num, int channel);
	~LewanSoulPlanner();
	void loop();
	int indexSplit=3;
	LewanSoulState_t state=StartupSerial;

	int targets[10]={0,};
	int positions[10]={0,};
};

#endif /* SRC_PID_LEWANSOULPLANNER_H_ */
