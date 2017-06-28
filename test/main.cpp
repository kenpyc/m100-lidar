#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include "../sdk/include/rplidar.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

// Constants
const int BATCH_SIZE = 8;	// number of values to be averaged in each batch. lower number = more precision and more noise

// Global driver variable
RPlidarDriver * drv;


// stops all the stuff and exits
void end(int) {

	// stop scanning
	drv->stop();

	// stop spinning
	drv->stopMotor();

	// dispose the driver variable from memory
	RPlidarDriver::DisposeDriver(drv);

	exit(0);
}

int main(int argc, const char * argv[]) {
	drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

	if (IS_FAIL(drv->connect("/dev/ttyUSB0", 115200))) {
		printf("Error, can't connect on that port.\n");
		exit(0);
	}

	// make SIGINT signal (ctrl-c) call end() instead of directly quitting
	signal(SIGINT, end);

	// Start spinning and scanning
	drv->startMotor();
	drv->startScan();

	// Start loop to get and print data
	while (true) {
		// Make array of measurements
		rplidar_response_measurement_node_t nodes[360];
		size_t count = _countof(nodes);

		// grab a 360 degree batch of data (IS_OK returns true if the operation worked)
		if (IS_OK(drv->grabScanData(nodes, count))) {

			// sort by angle
			drv->ascendScanData(nodes, count);

			// clear the console
			system("clear");

			// loop through and print angle, dist
			for (int i = 0; i < (int) count; i += BATCH_SIZE) {

				// Take the average of the batch, ignoring those with low quality
				float sum = 0;
				int nodes_summed = 0;
				for (int j = 0; j < BATCH_SIZE; j++) {
					if (nodes[i + j].sync_quality > 10) {
						sum += nodes[i + j].distance_q2 / 4.0f;
						nodes_summed++;
					}
				}

				// variable to store average distance. -1 means bad data
				float avg_distance = -1;

				if (nodes_summed > 0) {	// don't divide by 0
					avg_distance = sum / (float) nodes_summed;
				}

				// Print a . for every 25mm
				for (int j = 0; j < (int) (avg_distance / 25); j++) {
					printf(".");
				}
				printf("\n");
			}
		}
	}

	end(0);

	return 0;
}


