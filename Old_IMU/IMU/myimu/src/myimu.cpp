#include <freespace/freespace.h>
#include <freespace/freespace_printers.h>
#include <freespace/freespace_util.h>
#include <unistd.h>
#include <pthread.h>

#include <math.h>
#include "math/quaternion.h"
#include "math/vec3.h"
#include "math/quaternion.c"


#include <iostream>
#include <fstream>
#include <stdint.h>
#include <string>
#include <cstring>
#include <sys/time.h>
#include <time.h>
#include <iomanip>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>



using namespace std;

int stopped = 0;
void my_handler(int s){
    stopped = 1;

}

#define RADIANS_TO_DEGREES(rad) ((float) rad * (float) (180.0 / M_PI))
#define DEGREES_TO_RADIANS(deg) ((float) deg * (float) (M_PI / 180.0))

#define SLEEP    sleep(1)
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


int main (int argc, char *argv[]) {

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    // Set the filename
    char filename[255];
    struct tm* tm;
    time_t now;
    now = time(0);  //get current time
    tm = localtime(&now);   //get structure
    sprintf(filename, "imu_%02d%02d_%02d%02d.txt",tm->tm_mon+1, tm->tm_mday, tm->tm_hour, tm->tm_min);
    
    //string filename = "example.txt";
    //if(argc>1)
     //   filename = argv[1];
    
    struct InputLoopState inputLoop;
    struct freespace_MotionEngineOutput meOut;
    struct Vec3f eulerAngles;
    struct MultiAxisSensor accel, velocity;
    int rc;

    // Flag to indicate that the application should quit
    // Set by the control signal handler
    int quit = 0;

    //printVersionInfo(argv[0]);

    //addControlHandler(&quit);

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


    ofstream myfile (filename);
    if (myfile.is_open())
    {
        myfile << "Seq, Rot x, Rot y, Rot z, Vel x, Vel y, Vel z, Acc x, Acc y, Acc z, Time(m.d.y h:m:s.ms)\n";
        myfile.flush();
    }
    else {
        cout << "Unable to open file " << filename;
        exit(2);
    }
    unsigned int imu_seq = 0;
    cout << "Starting recording data from the sensor. Ctrl + C to stop" << endl;
    while(stopped==0)
    {
        rc = getMotionFromInputThread(&inputLoop, &meOut);

        if (rc) {

            getEulerAnglesFromMotion(&meOut, &eulerAngles);
            freespace_util_getAcceleration(&meOut, &accel);
            freespace_util_getAngularVelocity(&meOut, &velocity);
            

            //Get two times. One for date stamp and one for milliseconds
            time_t rawtime;
            time(&rawtime);
            struct tm * ptm;
            ptm = gmtime(&rawtime);
            struct timeval current_time;
            gettimeofday(&current_time, NULL);
            
            //Correcting Euler range (rad)
            if(eulerAngles.x < 0) eulerAngles.x *=-1;
                else eulerAngles.x = 2*M_PI - eulerAngles.x;
            if(eulerAngles.y < 0) eulerAngles.y *=-1;
                else eulerAngles.y = 2*M_PI - eulerAngles.y;
            if(eulerAngles.z < 0) eulerAngles.z *=-1;
                else eulerAngles.z = 2*M_PI - eulerAngles.z;

            eulerAngles.x = RADIANS_TO_DEGREES(eulerAngles.x);
            eulerAngles.y = RADIANS_TO_DEGREES(eulerAngles.y);
            eulerAngles.z = RADIANS_TO_DEGREES(eulerAngles.z);

            // Write data into the file
            // Extract orientation quaternion.
            myfile << imu_seq << ", ";
            myfile << eulerAngles.x << ", " << eulerAngles.y << ", " << eulerAngles.z << ", ";
            // Extract velocity vector
            myfile << velocity.x << ", " << velocity.y << ", " << velocity.z << ", ";
            // Extract acceleration vector
            myfile << accel.x << ", " << accel.y << ", " << accel.z << ", ";
            // Get date stamp
            myfile << ptm->tm_mon+1 << "." << ptm->tm_mday << "." << (1900+ptm->tm_year) << " " << ptm->tm_hour+2 << ":" << ptm->tm_min << ":" << ptm->tm_sec << ".";
            // Get milliseconds
            myfile << setfill('0') << setw(3) << current_time.tv_usec/1000 << endl;

            myfile.flush();
            // Same for cout

            cout << imu_seq << ", ";
            cout << eulerAngles.x << ", " << eulerAngles.y << ", " << eulerAngles.z << ", ";
            // Extract velocity vector
            cout << velocity.x << ", " << velocity.y << ", " << velocity.z << ", ";
            // Extract acceleration vector
            cout << accel.x << ", " << accel.y << ", " << accel.z << ", ";
            // Get date stamp
            cout << ptm->tm_mon+1 << "." << ptm->tm_mday << "." << (1900+ptm->tm_year) << " " << ptm->tm_hour+2 << ":" << setfill('0') << setw(2) << ptm->tm_min << ":" << ptm->tm_sec << ".";
            // Get milliseconds
            cout << setfill('0') << setw(3) << current_time.tv_usec/1000 << endl;
            imu_seq++;
        }
    }
    myfile.close();
    cout << "Done writing the file" << endl;
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

    // Put the device in the right operating mode
    memset(&message, 0, sizeof(message));
    message.messageType = FREESPACE_MESSAGE_DATAMODECONTROLV2REQUEST;
    message.dataModeControlV2Request.packetSelect = 8; // MEOut
    message.dataModeControlV2Request.mode = 4;         // Set full motion
    message.dataModeControlV2Request.formatSelect = 0; // MEOut format 0
    message.dataModeControlV2Request.ff0 = 1;          // Pointer fields
    message.dataModeControlV2Request.ff1 = 1;          // Pointer fields
    message.dataModeControlV2Request.ff2 = 1;          // Pointer fields
    message.dataModeControlV2Request.ff3 = 1;          // Angular velocity fields
    message.dataModeControlV2Request.ff4 = 1;          // Angular velocity fields
    message.dataModeControlV2Request.ff5 = 1;          // Angular velocity fields
    message.dataModeControlV2Request.ff6 = 1;          // Angular velocity fields
    message.dataModeControlV2Request.ff7 = 1;          // ActClass/PowerMgmt
    
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
    message.dataModeControlV2Request.packetSelect = 1;  // Mouse packets
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
