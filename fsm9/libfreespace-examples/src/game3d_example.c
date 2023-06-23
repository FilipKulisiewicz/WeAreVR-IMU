/**
 * This file is part of libfreespace-examples.
 *
 * Copyright (c) 2009-2013, Hillcrest Laboratories, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of the Hillcrest Laboratories, Inc. nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <freespace/freespace.h>
#include <freespace/freespace_util.h>
#include "appControlHandler.h"

#include <math.h>
#include "math/quaternion.h"
#include "math/vec3.h"

#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>


#ifdef WIN32
#include <windows.h>
#include "win32/pthread_win32.h"

#define M_PI    3.141592654
#else
#include <unistd.h>
#include <pthread.h>
#endif
/* */
// The sensor period to set the sensors to
#define SENSOR_PERIOD 2000 //10000
const char * const SENSOR_NAMES[] = {	
					"Accelerometer", 
					"Gyroscope", 
					"Magnetometer", 
					"Ambient Light Sensor",
					"Pressure Sensor",
					"Proximity Sensor",
					"Sensor Fusion"         };
/* */

// Cross platform sleep macro
#ifdef _WIN32
#define SLEEP    Sleep(100)
#else
#define SLEEP    sleep(1)
#endif
#define Sleep(x) usleep((x)*1000)

#define RADIANS_TO_DEGREES(rad) ((float) rad * (float) (180.0 / M_PI))
#define DEGREES_TO_RADIANS(deg) ((float) deg * (float) (M_PI / 180.0))

// State information for a thread
struct InputLoopState {
    pthread_t thread_;     // A handle to the thread
    pthread_mutex_t lock_; // A mutex to allow access to shared data
    int quit_;             // An input to the thread

    // The shared data updated by the thread
    struct freespace_MotionEngineOutput meOut_; // Motion data
    int updated_; // A flag to indicate that the motion data has been updated
};

// ============================================================================
// Local function prototypes
// ============================================================================
static void* inputThreadFunction(void*);
static int getMotionFromInputThread(struct InputLoopState* state,
                                    struct freespace_MotionEngineOutput* meOut);
static void getEulerAnglesFromMotion(const struct freespace_MotionEngineOutput* meOut,
                                     struct Vec3f* eulerAngles);

// ============================================================================
// Public functions
// ============================================================================

/******************************************************************************
 * main
 * This example uses the synchronous API to access the device.
 * It assumes the device is already connected. It uses pthreads to put the 
 * operation of handling incoming messages from the device in a separate thread
 * from the main thread. It configures the device to produce fused motion outputs.
 */
int main(int argc, char* argv[]) {
    struct InputLoopState inputLoop;
    struct freespace_MotionEngineOutput meOut;
    struct Vec3f eulerAngles;
    struct MultiAxisSensor accel;
    int rc; // Return code
    
    // Flag to indicate that the application should quit
    // Set by the control signal handler
    int quit = 0;

    /* User code 1 start */

    struct MultiAxisSensor angVel;
    struct MultiAxisSensor mag;
    int PrintMachineFriendly = 1;
    
    //printVersionInfo(argv[0]); //---- commented
    /* User code 1 end */
    
    addControlHandler(&quit);

    // Initialize the freespace library
    rc = freespace_init();
    if (rc != FREESPACE_SUCCESS) {
        printf("Initialization error. rc=%d\n", rc);
        exit(1);
    }
    
    // Setup the input loop thread
    memset(&inputLoop, 0, sizeof(struct InputLoopState));                      // Clear the state info for the thread
    pthread_mutex_init(&inputLoop.lock_, NULL);                                // Initialize the mutex
    pthread_create(&inputLoop.thread_, NULL, inputThreadFunction, &inputLoop); // Start the input thread
    
    // Give the device time to commit changes
	Sleep(1);
    /* */
    if(PrintMachineFriendly == 1){
        printf("sequenceNumber; roll; pitch; yaw; (accel:) x; y; z; (gyro:) X; Y; Z; (mag:) X; Y; Z \n");
    }
    /* */
    
    // Run the game loop
    while (!quit) {
        // Get input.
        rc = getMotionFromInputThread(&inputLoop, &meOut);
        
        // If new motion was available, use it
        if (rc) {
            // Run game logic.
            getEulerAnglesFromMotion(&meOut, &eulerAngles);
            freespace_util_getAcceleration(&meOut, &accel);
            /*  User code 3 start */
            freespace_util_getAngularVelocity(&meOut, &angVel);
            freespace_util_getMagnetometer(&meOut, &mag);
            // print
            if(PrintMachineFriendly == 1){
                printf("%d; %0.4f; %0.4f; %0.4f; %0.4f; %0.4f; %0.4f; % 6.2f; % 6.2f; % 6.2f; % 6.2f; % 6.2f; % 6.2f \n",
                   meOut.sequenceNumber,
                   RADIANS_TO_DEGREES(eulerAngles.x),
                   RADIANS_TO_DEGREES(eulerAngles.y),
                   RADIANS_TO_DEGREES(eulerAngles.z),
                   accel.x,
                   accel.y,
                   accel.z,
                   angVel.x, 
                   angVel.y,
                   angVel.z, 
                   mag.x,
                   mag.y,
                   mag.z);
            }
            else{
                printf("%d: roll: %0.4f, pitch: %0.4f, yaw: %0.4f\taccel\tx: %0.4f, y: %0.4f, z: %0.4f \t"
                   "gyro\t X: % 6.2f, Y: % 6.2f, Z: % 6.2f \t"
                   "mag\t X: % 6.2f, Y: % 6.2f, Z: % 6.2f \n",
                   meOut.sequenceNumber,
                   RADIANS_TO_DEGREES(eulerAngles.x),
                   RADIANS_TO_DEGREES(eulerAngles.y),
                   RADIANS_TO_DEGREES(eulerAngles.z),
                   accel.x,
                   accel.y,
                   accel.z,
                   /* added v */ 
                   angVel.x, 
                   angVel.y,
                   angVel.z, 
                   mag.x,
                   mag.y,
                   mag.z
                   );
            }
            /*  User code 3 end */
            fflush(stdout);
        }

        // Wait for "vsync"
        // SLEEP;
    }

    // Cleanup the input loop thread
    inputLoop.quit_ = 1;                     // Signal the thread to stop
    pthread_join(inputLoop.thread_, NULL);   // Wait until it does
    pthread_mutex_destroy(&inputLoop.lock_); // Get rid of the mutex

    // Finish using the library gracefully
    freespace_exit();

    return 0;
}

// ============================================================================
// Local functions
// ============================================================================

/******************************************************************************
 * getMotionFromInputThread
 *
 * @param state a pointer the the shared state information for the input loop thread
 * @param meOut a pointer to where to copy the motin information retrieved from the inpuit loop thread
 * @param return the updated flag from the input loop thread state
 */
static int getMotionFromInputThread(struct InputLoopState * state,
                                    struct freespace_MotionEngineOutput * meOut) {
    int updated;

    pthread_mutex_lock(&state->lock_);   // Obtain ownership of the input loop thread's shared state information
    *meOut = state->meOut_;              // Copy the motion packet to the main thread
    updated = state->updated_;           // Remember the updated_ flag
    state->updated_ = 0;                 // Mark the data as read
    pthread_mutex_unlock(&state->lock_); // Release ownership of the input loop thread's shared state information

    return updated;
}

/******************************************************************************
 * getEulerAnglesFromMotion
 */
static void getEulerAnglesFromMotion(const struct freespace_MotionEngineOutput* meOut,
                                     struct Vec3f* eulerAngles) {
    struct MultiAxisSensor sensor;
    struct Quaternion q;

    // Get the angular position data from the MEOut packet
    freespace_util_getAngPos(meOut, &sensor);

    // Copy the data over to because both the util API and quaternion.h each have their own structs
    q.w = sensor.w;
    q.x = sensor.x;
    q.y = sensor.y;
    q.z = sensor.z;

    // The Freespace quaternion gives the rotation in terms of
    // rotating the world around the object. We take the conjugate to
    // get the rotation in the object's reference frame.
    q_conjugate(&q, &q);

    // Convert quaternion to Euler angles
    q_toEulerAngles(eulerAngles, &q);
}

/**
 * sendSetSensorPeriodMessage
 * Sends a message to change the sample rate
 * of a sensor on a Freespace device.
 * device - The Freespace Device ID of the device
 * sensor - The sensor index - see the HCOMM document
 * for more information
 * period - The desired period of the sensor in us.
 * commit - 0 to write without commit, 1 to commit changes
 * return - The return code of the message
 */
int sendSetSensorPeriodMessage(FreespaceDeviceId device, int sensor, int period, int commit) {
	struct freespace_message message;

	// Send the sensor period message with the user parameters
	memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

	message.messageType = FREESPACE_MESSAGE_SENSORPERIODREQUEST;
	message.sensorPeriodRequest.commit = commit;	// Need to commit change in order to work
	message.sensorPeriodRequest.get = 0;  // We are setting, not getting
	message.sensorPeriodRequest.sensor = sensor; // Sensor index - see the HCOMM doc for more info
	message.sensorPeriodRequest.period = period; // Period in us

	return freespace_sendMessage(device, &message);
}

/**
 * sendGetSensorPeriodMessage
 * Sends a message to read the sample rate
 * of a sensor on a Freespace device.
 * device - The Freespace Device ID of the device
 * sensor - The sensor index - see the HCOMM document
 * for more information
 * return - The return code of the message
 */
int sendGetSensorPeriodMessage(FreespaceDeviceId device, int sensor) {
	struct freespace_message message;

	// Send the sensor period message with the user parameters
	memset(&message, 0, sizeof(message)); // Make sure all the message fields are initialized to 0.

	message.messageType = FREESPACE_MESSAGE_SENSORPERIODREQUEST;
	message.sensorPeriodRequest.get = 1;  // We are getting, not setting
	message.sensorPeriodRequest.sensor = sensor; // Sensor index - see HCOMM doc for more info

	return freespace_sendMessage(device, &message);
}

/**
 * waitForPeriodResponse
 * Waits on a freespace message response to a
 * sensor period message. Waits up to MAX_WAIT_SECS
 * before giving up.
 * device - The Freespace Device ID of the device
 * sensorValue - Pointer where the sensor index is stored
 * periodValue - Pointer where the period value is stored
 * return - FREESPACE_SUCCESS if successful, or a FREESPACE_ERROR otherwise
 */
int waitForPeriodResponse(FreespaceDeviceId device, int* sensorValue, int* periodValue) {
	int rc = 0;
	struct freespace_message message;

	// Keep looping if we get FREESPACE_SUCCESS but no SensorPeriodResponse
	while (rc == FREESPACE_SUCCESS) {
		rc = freespace_readMessage(device, &message, 200);
		if (rc != FREESPACE_SUCCESS) {
			return rc;
		}
		// Check if the sensor has given us a Sensor Period response
		if (message.messageType == FREESPACE_MESSAGE_SENSORPERIODRESPONSE) {
			if (sensorValue != NULL)
				*sensorValue = message.sensorPeriodResponse.sensor;
			if (periodValue != NULL)
				*periodValue = message.sensorPeriodResponse.period;
			return FREESPACE_SUCCESS;
		}
	}
	return 0;
}

/**
 * printSensorInfo
 * Prints the sensor period information for a device's sensors.
 * It sends a getSensorPeriod message for every sensor, then waits
 * for the responses, each containing the period for a sensor.
 * device - The Freespace Device ID of the device
 * return - FREESPACE_SUCCESS if successful, or a FREESPACE_ERROR otherwise
 */
int printSensorInfo(FreespaceDeviceId device) {
	int rc;
	int index;
	int sensor;
	int period;

	// Update sensor information
	printf("\nSensors:\n");
	for (index = 0;index < 7;index++) {
		// Request the sensor period information
		rc = sendGetSensorPeriodMessage(device, index);
		if (rc != FREESPACE_SUCCESS) {
			return rc;
		}

		// Wait for a response
		rc = waitForPeriodResponse(device, &sensor, &period);

		if (rc == FREESPACE_ERROR_TIMEOUT) { // Indicates timeout
			printf("     %d. %s TIMED OUT.\n", index, SENSOR_NAMES[index]);
		} else if (rc == FREESPACE_SUCCESS) {
			printf("     %d. %s", sensor, SENSOR_NAMES[index]);
			if (period != 0)
				printf(" @ %d us.\n", period);
			else
				printf(" disabled.\n");
		} else {
			return rc;
		}
	}
	return FREESPACE_SUCCESS;
	printf("\n");
}

// ============================================================================
// Thread functions
// ============================================================================

/******************************************************************************
 * inputThreadFunction
 */
static void* inputThreadFunction(void* arg) {
    struct InputLoopState* state = (struct InputLoopState*) arg;
    struct freespace_message message;
    FreespaceDeviceId device;
    int numIds;
    int rc;
    /* */
    int i = 0;
    int period = 0; // Holds the period of the sensor
    int sensor = 0;	// Holds the sensor number
    /* */
    
    /** --- START EXAMPLE INITIALIZATION OF DEVICE -- **/
    // This example requires that the freespace device already be connected
    // to the system before launching the example.
    rc = freespace_getDeviceList(&device, 1, &numIds);
    if (numIds == 0) {
        printf("freespaceInputThread: Didn't find any devices.\n");
        exit(1);
    }

    rc = freespace_openDevice(device);
    if (rc != FREESPACE_SUCCESS) {
        printf("freespaceInputThread: Error opening device: %d\n", rc);
        exit(1);
    }

    rc = freespace_flush(device);
    if (rc != FREESPACE_SUCCESS) {
        printf("freespaceInputThread: Error flushing device: %d\n", rc);
        exit(1);
    }

    /* User code 2 end */
    // Set all sensors to SENSOR_PERIOD [um]
	for (i = 0;i < 7;i++) {
		if (i == 6) {	// If we are on last sensor we need to commit
			rc = sendSetSensorPeriodMessage(device, i, SENSOR_PERIOD, 1);
		} else {
			rc = sendSetSensorPeriodMessage(device, i, SENSOR_PERIOD, 0);
		}
		if (rc != FREESPACE_SUCCESS) {
			printf("Could not send message: %d.\n", rc);
            exit(1);
		}
		
		// Wait for a response to the change
		rc = waitForPeriodResponse(device, &sensor, &period);
		if (rc == FREESPACE_ERROR_TIMEOUT) {
			printf("%s timed out.\n", SENSOR_NAMES[i]);
		} else if (rc != FREESPACE_SUCCESS) {
			printf("Failed with error code: %d.\n", rc);
            exit(1);
		}
	}
	
    // Give the device time to commit changes
	Sleep(1);

    // Print out the new sensor info
	rc = printSensorInfo(device);
	if (rc != FREESPACE_SUCCESS) {
		printf("Error getting sensor info: %d\n", rc);
        exit(1);
	}

	printf("\n");
    /* User code 2 end */

    // Put the device in the right operating mode
    memset(&message, 0, sizeof(message));
    message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
    message.dataModeControlV2Request.packetSelect = 8; // MEOut
    message.dataModeControlV2Request.mode = 4;         //changed from 0;         // Set full motion on (always on?)
    message.dataModeControlV2Request.formatSelect = 0; // MEOut format 0
    message.dataModeControlV2Request.ff1 = 1;          // Acceleration fields
    message.dataModeControlV2Request.ff6 = 1;          // Angular (orientation) fields
    /* User code begin 4 */
    //gyro
    //message.dataModeControlV2Request.ff0 = 1;         // Pointer fields
    message.dataModeControlV2Request.ff3 = 1;           // Angular velocity fields 
    //mag
    message.dataModeControlV2Request.ff4 = 1;           // Magnetometer fields 
    //Power Managment
    message.dataModeControlV2Request.ff7 = 0;          // ActClass/PowerMgmt
    
    /* User code end 4 */
    
    rc = freespace_sendMessage(device, &message);
    if (rc != FREESPACE_SUCCESS) {
        printf("freespaceInputThread: Could not send message: %d.\n", rc);
    }
    /** --- END EXAMPLE INITIALIZATION OF DEVICE -- **/

    // The input loop
    while (!state->quit_) {
        rc = freespace_readMessage(device, &message, 1000 /* 1 second timeout */);
        if (rc == FREESPACE_ERROR_TIMEOUT ||
            rc == FREESPACE_ERROR_INTERRUPTED) {
            continue;
        }
        if (rc != FREESPACE_SUCCESS) {
            printf("freespaceInputThread: Error reading: %d. Trying again after a second...\n", rc);
            SLEEP;
            continue;
        }

        // Check if this is a MEOut message.
        if (message.messageType == FREESPACE_MESSAGE_MOTIONENGINEOUTPUT) {
            pthread_mutex_lock(&state->lock_);
            // Update state fields.
            state->meOut_ = message.motionEngineOutput;
            state->updated_ = 1;

            pthread_mutex_unlock(&state->lock_);
        }
    }

    /** --- START EXAMPLE FINALIZATION OF DEVICE --- **/
    printf("\n\nfreespaceInputThread: Cleaning up...\n");
    memset(&message, 0, sizeof(message));
    message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
    message.dataModeControlV2Request.packetSelect = 1;  // Mouse packets = 1 
    message.dataModeControlV2Request.mode = 0;          // Set full motion
    rc = freespace_sendMessage(device, &message);
    if (rc != FREESPACE_SUCCESS) {
        printf("freespaceInputThread: Could not send message: %d.\n", rc);
    }

    freespace_closeDevice(device);
    /** --- END EXAMPLE FINALIZATION OF DEVICE --- **/
    
    // Exit the thread.
    return 0;
}
