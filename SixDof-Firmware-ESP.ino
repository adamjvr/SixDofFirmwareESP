/**@file template.ino */
#include <SimplePacketComs.h>
#include <lx16a-servo.h>
#include <Esp32WifiManager.h>
#include <Esp32SimplePacketComs.h>
#include <server/NameCheckerServer.h>
#include "src/commands/GetPIDData.h"
#include "src/commands/SetPIDSetpoint.h"

//Create a wifi manager
WifiManager manager;
UDPSimplePacket coms;
LewanSoulPlanner planner(7);
String name ="GroguArm";
void setup() {

	manager.setup();
	coms.attach(new SetPIDSetpoint(7,&planner));
	coms.attach(new GetPIDData(7,&planner));
	coms.attach(new NameCheckerServer(&name));
}

void loop() {
	// read the serial port for new passwords and maintain connections
	manager.loop();
	planner.loop();
	if (manager.getState() == Connected) {
		// use the Wifi Stack now connected
		coms.server();
	}
}
