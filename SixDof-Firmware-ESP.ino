/**@file template.ino */
#include <SimplePacketComs.h>
#include <lx16a-servo.h>
#include <Esp32WifiManager.h>
#include <Esp32SimplePacketComs.h>
#include <server/NameCheckerServer.h>
#include "src/commands/GetPIDData.h"
#include "src/commands/SetPIDSetpoint.h"

//Create a wifi manager
static WifiManager manager;
static UDPSimplePacket coms;
static LewanSoulPlanner * planner=NULL;
static String name ="GroguArm";
static TaskHandle_t complexHandlerTaskUS;
void MotorThread(void * param){
	Serial.println("Starting the Motor loop thread");

	while (1) {
		vTaskDelay(1); //sleep 10ms
		if(planner!=NULL)
			planner->loop();
	}
	Serial.println("ERROR Motor thread died!");
}

void setup() {
	planner =  new LewanSoulPlanner(7);
	manager.setup();
	coms.attach(new SetPIDSetpoint(7,planner));
	coms.attach(new GetPIDData(7,planner));
	coms.attach(new NameCheckerServer(&name));
	xTaskCreatePinnedToCore(MotorThread, "Motor Thread", 8192, NULL, 1, // low priority timout thread
					&complexHandlerTaskUS, 1);
}

void loop() {
	// read the serial port for new passwords and maintain connections
	manager.loop();
	if (manager.getState() == Connected) {
		// use the Wifi Stack now connected
		coms.server();
	}
}
