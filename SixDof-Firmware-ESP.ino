/**@file template.ino */
#include <SimplePacketComs.h>
#include <lx16a-servo.h>
#include <Esp32WifiManager.h>
#include <Esp32SimplePacketComs.h>
#include "src/commands/GetPIDData.h"
#include "src/commands/SetPIDSetpoint.h"

//Create a wifi manager
WifiManager manager;
UDPSimplePacket coms;
LewanSoulPlanner planner(7);
void setup() {

	Serial.begin(115200);

	Serial.println("Starting");
	manager.setup();
	coms.attach(new SetPIDSetpoint(7,&planner));
	coms.attach(new GetPIDData(7,&planner));

}

void loop() {
	// read the serial port for new passwords and maintain connections
	manager.loop();
	if (manager.getState() == Connected) {
		planner.loop();
		// use the Wifi Stack now connected
		coms.server();
	}
}
