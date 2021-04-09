// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>

#include "luatables.h"
#include "OPKinematics.h"

#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include "messages.pb.h"
#if GOOGLE_PROTOBUF_VERSION < 3006001
#define ByteSizeLong ByteSize
#endif

static void close_socket(int fd) {
#ifdef _WIN32
	closesocket(fd);
#else
	close(fd);
#endif
}

struct sockaddr_in address;
struct hostent *server;
int fd, rc;
char *buffer;
int port = 10003;
char host[256];  // localhost
int n;

#define NMOTORS 20
static const char *jointNames[NMOTORS] = {"Neck", "Head",
                                          "ShoulderL", "ArmUpperL", "ArmLowerL",
                                          "PelvYL", "PelvL", "LegUpperL", "LegLowerL", "AnkleL", "FootL", 
                                          "PelvYR", "PelvR", "LegUpperR", "LegLowerR", "AnkleR", "FootR",
                                          "ShoulderR", "ArmUpperR", "ArmLowerR",
                                         };
std::vector<int> moveDir(NMOTORS, 0);
std::vector<int> jointReverse;

// Walk Parameters
// Stance and velocity limit values
std::vector<double> stanceLimitX(2, 0);
std::vector<double> stanceLimitY(2, 0);
std::vector<double> stanceLimitA(2, 0);
std::vector<double> velDelta(3, 0);

double velXHigh;
double velDeltaXHigh;

// Toe/heel overlap checking values
std::vector<double> footSizeX(2, 0);
double stanceLimitMarginY;
double stanceLimitY2;

// Stance parameters
double bodyHeight;
double bodyTilt;
double footX;
double footY;
double supportX;
double supportY;
std::vector<double> qLArm0(3, 0);
std::vector<double> qRArm0(3, 0);

double hardnessSupport;
double hardnessSwing;

// Gait parameters
double tStep0;
double tStep;
double tZmp;
double stepHeight0;
double stepHeight;
double ph1Single, ph2Single;
double ph1Zmp, ph2Zmp;

// Compensation parameters
double hipRollCompensation;
std::vector<double> ankleMod(2, 0);
double turnCompThreshold;
double turnComp;

// Gyro stabilization parameters
std::vector<double> ankleImuParamX(4, 0);
std::vector<double> ankleImuParamY(4, 0);
std::vector<double> kneeImuParamX(4, 0);
std::vector<double> hipImuParamY(4, 0);
std::vector<double> armImuParamX(4, 0);
std::vector<double> armImuParamY(4, 0);

// Support bias parameters to reduce backlash-based instability
double velFastForward;
double velFastTurn;
double supportFront;
double supportFront2;
double supportBack;
double supportSideX;
double supportSideY;
double supportTurn;

double frontComp;
double AccelComp;

// Initial body swing
double supportModYInitial;

// WalkKick parameters
double walkKickPh;
double toeTipCompensation;


// Walk state variables
std::vector<double> uTorso(3, 0);
std::vector<double> uLeft(3, 0);
std::vector<double> uRight(3, 0);

std::vector<double> uTorso1(3, 0);
std::vector<double> uTorso2(3, 0);
std::vector<double> uLeft1(3, 0);
std::vector<double> uLeft2(3, 0);
std::vector<double> uRight1(3, 0);
std::vector<double> uRight2(3, 0);
std::vector<double> uSupport(3, 0);

std::vector<double> pLLeg;
std::vector<double> pRLeg;
std::vector<double> pTorso;

std::vector<double> velCurrent(3, 0);
std::vector<double> velCommand(3, 0);
std::vector<double> velDiff(3, 0);

// ZMP exponential coefficients:
double aXP, aXN, aYP, aYN;

// Gyro stabilization variables
std::vector<double> ankleShift(2, 0);
double kneeShift;
std::vector<double> hipShift(2, 0);
std::vector<double> armShift(2, 0);

bool active;
bool started;
int iStep0;
int iStep;
double t0;
double tLastStep;

double ph0;
double ph;

int stopRequest;
int walkKickRequest;
int current_step_type;

int initial_step;

int upper_body_overridden;
int motion_playing;

std::vector<double> qLArmOR0(3, 0);
std::vector<double> qRArmOR0(3, 0);

std::vector<double> qLArmOR(3, 0);
std::vector<double> qRArmOR(3, 0);

std::vector<double> bodyRot(3, 0);
double phSingle;

// Standard offset
std::vector<double> uLRFootOffset(3, 0);

// Walking/Stepping transition variables
std::vector<double> uLeftI(3, 0);
std::vector<double> uRightI(3, 0);
std::vector<double> uTorsoI(3, 0);
int supportI;
bool start_from_step;

// comdot = {0, 0}
int stepkick_ready;
int stepKickRequest;
int has_ball;
int supportLeg;
std::vector<double> supportMod(2, 0);
double shiftFactor;
double m1X;
double m2X;
double m1Y;
double m2Y;
std::vector<double> uRight15(3, 0);
std::vector<double> uLeft15(3, 0);
std::vector<double> uTorsoActual(3, 0);

std::vector<std::vector<std::vector<double>>> kickDefLeft, kickDefRight;
int kickState;
std::vector<double> uBody1(3, 0);
std::vector<double> uBody(3, 0);
std::vector<double> qLArm1(3, 0);
double zLeft1, zLeft, zRight1, zRight;
double aLeft1, aLeft, aRight1, aRight;
double bodyRoll, bodyRoll1, bodyPitch, bodyPitch1, zBody1, zBody;
double torsoShiftX;
double kickXComp;
double qRHipRollCompensation, qLHipRollCompensation;
std::vector<double> supportCompL(3, 0);
std::vector<double> supportCompR(3, 0);
bool done;

static char *read_file(const char *filename) {
	char *buffer = NULL;
	FILE *fp = fopen(filename, "r");
	if (!fp)
	return NULL;
	if (fseek(fp, 0L, SEEK_END) == 0) {
	const long size = ftell(fp);
	assert(size != -1);
	buffer = (char *)malloc(sizeof(char) * (size + 1));
	fseek(fp, 0L, SEEK_SET);
	const size_t len = fread(buffer, sizeof(char), size, fp);
	buffer[len] = '\0';
	}
	fclose(fp);
	return buffer;
}

static void socket_closed_exit() {
	printf("Connection closed by server.\n");
	exit(1);
}

template<class T>
SensorMeasurements sendMessage(T message) {
    #ifndef _WIN32
            // This doesn't work on Windows, we should implement SocketOutputStream to make it work efficiently on Windows
            // See https://stackoverflow.com/questions/23280457/c-google-protocol-buffers-open-http-socket
            const int size = htonl(message.ByteSizeLong());
            if (send(fd, (char *)(&size), sizeof(int), 0) == -1)
                socket_closed_exit();
            google::protobuf::io::ZeroCopyOutputStream *zeroCopyStream = new google::protobuf::io::FileOutputStream(fd);
            message.SerializeToZeroCopyStream(zeroCopyStream);
            delete zeroCopyStream;
    #else // here we make a useless malloc, copy and free
            const int size = message.ByteSizeLong();
            char *output = (char *)malloc(sizeof(int) + size);
            int *output_size = (int *)output;
            *output_size = htonl(size);
            message.SerializeToArray(&output[sizeof(int)], size);
            if (send(fd, output, sizeof(int) + size, 0) == -1)
            {
                free(output);
                socket_closed_exit();
            }
            free(output);
    #endif
    int s;
    if (recv(fd, (char *)&s, sizeof(int), 0) == -1)
        socket_closed_exit();
    const int answer_size = ntohl(s);
    SensorMeasurements sensorMeasurements;
    if (answer_size)
    {
        buffer = (char *)malloc(answer_size);
        int i = 0;
        while (i < answer_size)
        {
            n = recv(fd, &buffer[i], answer_size, 0);
            if (n == -1)
                socket_closed_exit();
            i += n;
        }
        sensorMeasurements.ParseFromArray(buffer, answer_size);
        free(buffer);
    }
    // std::string printout;
    // google::protobuf::TextFormat::PrintToString(sensorMeasurements, &printout);
    // std::cout << printout << std::endl;
    return sensorMeasurements;
}

double getTime() {
	ActuatorRequests aR;
	MotorPosition *mp = aR.add_motor_positions();
	mp->set_name("Neck");
	mp->set_position(0);
	SensorMeasurements sM = sendMessage(aR);
	return sM.time();
}

void set_actuator_command(double command, int index) {
    ActuatorRequests actuatorRequests;
    MotorPosition *mp = actuatorRequests.add_motor_positions();
    mp->set_name(jointNames[index]);
    mp->set_position(moveDir[index] * command);

    sendMessage(actuatorRequests);
}

void set_actuator_command(std::vector<double> commands, int index) {
    ActuatorRequests actuatorRequests;

    for (int i = 0; i < int(commands.size()); i++) {
        MotorPosition *mp = actuatorRequests.add_motor_positions();
        mp->set_name(jointNames[index + i]);
        mp->set_position(moveDir[index + i] * commands[i]);
    }
    
    sendMessage(actuatorRequests);
}

void set_lleg_command(std::vector<double> val) {
    set_actuator_command(val, 5);
}

double mod_angle(double a) {
    a = remainder(a, 2 * M_PI);
    if (a >= M_PI) {
        a = a - 2 * M_PI;
    }
    return a;
}

std::vector<double> pose_global(std::vector<double> pRelative, std::vector<double> pose) {
    double ca = cos(pose[2]);
    double sa = sin(pose[2]);
    std::vector<double> GlobalPose(3, 0);
    GlobalPose[0] = pose[0] + ca*pRelative[0] - sa*pRelative[1];
    GlobalPose[1] = pose[1] + sa*pRelative[0] + ca*pRelative[1];
    GlobalPose[2] = pose[2] + pRelative[2];
    return GlobalPose;
}

std::vector<double> pose_relative(std::vector<double> pGlobal, std::vector<double> pose) {
    double ca = cos(pose[2]);
    double sa = sin(pose[2]);
    double px = pGlobal[0] - pose[0];
    double py = pGlobal[1] - pose[1];
    double pa = pGlobal[2] - pose[2];
    std::vector<double> LocalPose(3, 0);
    LocalPose[0] = ca * px + sa * py;
    LocalPose[1] = -sa * px + ca * py;
    LocalPose[2] = mod_angle(pa);
    return LocalPose;
}

std::vector<double> se2_interpolate(double t, std::vector<double> u1, std::vector<double> u2) {
    // helps smooth out the motions using a weighted average
    std::vector<double> out(3, 0);
    out[0] = u1[0] + t * (u2[0] - u1[0]);
    out[1] = u1[1] + t * (u2[1] - u1[1]);
    out[2] = u1[2] + t * mod_angle(u2[2] - u1[2]);
    return out;
}

void motion_legs(std::vector<double> qLegs, bool gyro_off = false) {
    set_lleg_command(qLegs);
}

std::vector<double> inverse_legs(std::vector<double> pLLeg, std::vector<double> pRLeg, std::vector<double> pTorso, int supportLeg) {
    std::vector<double> qLegs;
    int leg = 0;
    qLegs = darwinop_kinematics_inverse_legs(&pLLeg[0], 
				                             &pRLeg[0],
				                             &pTorso[0], leg);
    return qLegs;
}

void entry() {
    
    torsoShiftX = 0;

    started = false;
    active = true;

    // Initialize state variables
    uLeft = {-supportX, footY, 0};
    uRight = {-supportX, -footY, 0};
    uLeft1 = {-supportX, footY, 0};
    uRight1 = {-supportX, -footY, 0};
    uBody = {0, 0, 0};
    uBody1 = {0, 0, 0};
    
    zLeft = 0;
    zRight = 0;
    zLeft1 = 0;
    zRight1 = 0;
    // foot height
    
    
    aLeft = 0;
    aRight = 0;
    aLeft1 = 0;
    aRight1 = 0;
    // foot pitch angle
    
    zBody = bodyHeight;
    zBody1 = bodyHeight;

    bodyRoll = 0;
    bodyRoll1 = 0;
    
    bodyPitch = bodyTilt;
    bodyPitch1 = bodyTilt;
    qLHipRollCompensation = 0;
    qRHipRollCompensation = 0;
    done = false;
}

void update() {

    if (!started) {
        started = true;
        kickState = 1;
        t0 = getTime();
    }

    double t = getTime();
    ph = (t - t0) / kickDefLeft[kickState-1][1][0];
    if (ph > 1) {
        kickState = kickState + 1;
        
        uLeft1[0] = uLeft[0];
        uLeft1[1] = uLeft[1];
        uLeft1[2] = uLeft[2];
        
        uRight1[1] = uRight[0];
        uRight1[2] = uRight[1];
        uRight1[3] = uRight[2];
        
        uBody1[0] = uBody[0];
        uBody1[1] = uBody[1];
        uBody1[2] = uBody[2];
        
        // qLArm1[0] = qLArm[0];
        // qLArm1[1] = qLArm[1];
        // qLArm1[2] = qLArm[2];

        // qRArm1[0] = qRArm[0];
        // qRArm1[1] = qRArm[1];
        // qRArm1[2] = qRArm[2];
    
        zLeft1 = zLeft;
        zRight1 = zRight;
        
        
        aLeft1 = aLeft;
        aRight1 = aRight;
        
        
        
        bodyRoll1 = bodyRoll;
        bodyPitch1 = bodyPitch;
        zBody1 = zBody;

        if (kickState > int(kickDefLeft.size())) {
            done = true;
        }

        ph = 0;
        t0 = getTime();
    }

    if (!done) {

        int kickStepType = kickDefLeft[kickState-1][0][0];

        // Tosro X position offxet (for differetly calibrated robots)
        if (kickState == 1) { // Initial slide
            // elseif kickState == #kickDef-1 then
            torsoShiftX = kickXComp * ph;
        } else if (kickState == int(kickDefLeft.size())) {
            torsoShiftX = kickXComp * (1 - ph);
        }

        if (kickState == 2) { // Lift step
            if (kickStepType == 2) {
                // Instant roll compensation
                //      qRHipRollCompensation= -hipRollCompensation*ph;
                qRHipRollCompensation = -hipRollCompensation;
            } else if (kickStepType == 3) {
                //      qLHipRollCompensation= hipRollCompensation*ph;
                qLHipRollCompensation = hipRollCompensation;
            }
        } else if (kickState == int(kickDefLeft.size())) { // Final step
            if (qRHipRollCompensation < 0) {
                qRHipRollCompensation = -hipRollCompensation * (1 - ph);
            } else if (qLHipRollCompensation > 0) {
                qLHipRollCompensation = hipRollCompensation * (1 - ph);
            }
        }

        if (kickStepType == 1) {
            uBody = se2_interpolate(ph, uBody1, kickDefLeft[kickState-1][2]);
            if (kickDefLeft[kickState-1].size() >= 4) {
                zBody = ph * kickDefLeft[kickState-1][3][0] + (1 - ph) * zBody1;
            }
            if (kickDefLeft[kickState-1].size() >= 5) {
                bodyRoll = ph * kickDefLeft[kickState-1][4][0] + (1 - ph) * bodyRoll1;
            }
            if (kickDefLeft[kickState-1].size() >= 6) {
                bodyPitch = ph * kickDefLeft[kickState-1][5][0] + (1 - ph) * bodyPitch1;
            }
        } else if (kickStepType == 2) { // Lifting / Landing Left foot
            uBody = se2_interpolate(ph, uBody1, kickDefLeft[kickState-1][2]);
            uLeft = se2_interpolate(ph, uLeft1, pose_global(kickDefLeft[kickState-1][3], uLeft1));
            zLeft = ph * kickDefLeft[kickState-1][4][0] + (1 - ph) * zLeft1;
            aLeft = ph * kickDefLeft[kickState-1][5][0] + (1 - ph) * aLeft1;
            // aLeft = kickDef[kickState][6]
            if (kickDefLeft[kickState-1].size() >= 7) {
                bodyPitch = ph * kickDefLeft[kickState-1][6][0] + (1 - ph) * bodyPitch1;
            }
        } else if (kickStepType == 3) { // Lifting / Landing Right foot
            uBody = se2_interpolate(ph, uBody1, kickDefLeft[kickState-1][2]);
            uRight = se2_interpolate(ph, uRight1, pose_global(kickDefLeft[kickState-1][3], uRight1));
            zRight = ph * kickDefLeft[kickState-1][4][0] + (1 - ph) * zRight1;
            aRight = ph * kickDefLeft[kickState-1][5][0] + (1 - ph) * aRight1;
            // aRight =  kickDef[kickState][6]
            if (kickDefLeft[kickState-1].size() >= 7) {
                bodyPitch = ph * kickDefLeft[kickState-1][6][0] + (1 - ph) * bodyPitch1;
            }
        } else if (kickStepType == 4) { // Kicking Left foot
            uBody = se2_interpolate(ph, uBody1, kickDefLeft[kickState-1][2]);
            uLeft = se2_interpolate(ph, uLeft1, kickDefLeft[kickState-1][3]);
            zLeft = ph * kickDefLeft[kickState-1][4][0] + (1 - ph) * zLeft1;
            aLeft = kickDefLeft[kickState-1][5][0];
            // aLeft = ph * kickDef[kickState][6] + (1 - ph) * aLeft1
            if (kickDefLeft[kickState-1].size() >= 7) {
                bodyPitch = ph * kickDefLeft[kickState-1][6][0] + (1 - ph) * bodyPitch1;
            }
        } else if (kickStepType == 5) { // Kicking Right foot
            uBody = se2_interpolate(ph, uBody1, kickDefLeft[kickState-1][2]);
            uRight = se2_interpolate(ph, uRight1, kickDefLeft[kickState-1][3]);
            zRight = ph * kickDefLeft[kickState-1][4][0] + (1 - ph) * zRight1;
            // aRight = ph * kickDef[kickState][6] + (1 - ph) * aRight1
            aRight = kickDefLeft[kickState-1][5][0];
            if (kickDefLeft[kickState-1].size() >= 7) {
                bodyPitch = ph * kickDefLeft[kickState-1][6][0] + (1 - ph) * bodyPitch1;
            }
        } else if (kickStepType == 6) { //Returning to walk stance
            uBody = se2_interpolate(ph, uBody1, kickDefLeft[kickState-1][2]);
            zBody = ph * bodyHeight + (1 - ph) * zBody1;
            bodyRoll = (1 - ph) * bodyRoll1;
            if (kickDefLeft[kickState-1].size() >= 4) {
                bodyPitch = ph * kickDefLeft[kickState-1][3][0] + (1 - ph) * bodyPitch1;
            }
            // qLArm = {qLArm0[0], qLArm0[1], qLArm0[2]};
            // qRArm = {qRArm0[0], qRArm0[1], qRArm0[2]};
        } else if (kickStepType == 7) { // Upper body movement
            uBody = se2_interpolate(ph, uBody1, kickDefLeft[kickState-1][2]);
            // qLArm = se2_interpolate(ph, qLArm1, kickDefLeft[kickState-1][3]);
            // qRArm = se2_interpolate(ph, qRArm1, kickDefLeft[kickState-1][4]);
            if (kickDefLeft[kickState-1].size() >= 6) {
                zBody = ph * kickDefLeft[kickState-1][5][0] + (1 - ph) * zBody1;
            }
            if (kickDefLeft[kickState-1].size() >= 7) {
                bodyRoll = ph * kickDefLeft[kickState-1][6][0] + (1 - ph) * bodyRoll1;
            }
            if (kickDefLeft[kickState-1].size() >= 8) {
                bodyPitch = ph * kickDefLeft[kickState-1][7][0] + (1 - ph) * bodyPitch1;
            }
        }

        std::vector<double> uLeftActual = pose_global(supportCompL, uLeft);
        std::vector<double> uRightActual = pose_global(supportCompR, uRight);
        uTorso = pose_global({-footX - torsoShiftX, 0, 0}, uBody);
        pLLeg =
            {
                uLeftActual[0],
                uLeftActual[1],
                zLeft,
                0,
                aLeft,
                uLeftActual[2]
            };
        
        pRLeg = {
            uRightActual[0],
            uRightActual[1],
            zRight,
            0,
            aRight,
            uRightActual[2]
            };
        
        pTorso = {
                uTorso[0],
                uTorso[1],
                zBody,
                bodyRoll,
                bodyPitch,
                uTorso[2]
            };

        std::vector<double> qLegs = inverse_legs(pLLeg, pRLeg, pTorso, 0);
        motion_legs(qLegs);
        // motion_arms()
    }

}

void InitializeVariablesFromConfig() {
    LuaTable walkConfig (LuaTable::fromFile("Config_Walk.lua"));
    stanceLimitX[0] = walkConfig["stanceLimitX"][1].getDefault<double>(false);
    stanceLimitX[1] = walkConfig["stanceLimitX"][2].getDefault<double>(false);

    stanceLimitY[0] = walkConfig["stanceLimitY"][1].getDefault<double>(false);
    stanceLimitY[1] = walkConfig["stanceLimitY"][2].getDefault<double>(false);
    
    stanceLimitA[0] = walkConfig["stanceLimitA"][1].getDefault<double>(false);
    stanceLimitA[1] = walkConfig["stanceLimitA"][2].getDefault<double>(false);
    
    velDelta[0] = walkConfig["velDelta"][1].getDefault<double>(false);
    velDelta[1] = walkConfig["velDelta"][2].getDefault<double>(false);
    velDelta[2] = walkConfig["velDelta"][3].getDefault<double>(false);
    
    velXHigh = walkConfig["velXHigh"].getDefault<double>(false);
    velDeltaXHigh = walkConfig["velDeltaXHigh"].getDefault<double>(false);
    
    footSizeX[0] = walkConfig["footSizeX"][1].getDefault<double>(false);
    footSizeX[1] = walkConfig["footSizeX"][2].getDefault<double>(false);

    stanceLimitMarginY = walkConfig["stanceLimitMarginY"].getDefault<double>(false);
    stanceLimitY2 = 2 * walkConfig["footY"].getDefault<double>(false) - stanceLimitMarginY;
    
    bodyHeight = walkConfig["bodyHeight"].getDefault<double>(false);
    bodyTilt = walkConfig["bodyTilt"].getDefault<double>(false);
    footX = walkConfig["footX"].getDefault<double>(false);
    footY = walkConfig["footY"].getDefault<double>(false);
    supportX = walkConfig["supportX"].getDefault<double>(false);
    supportY = walkConfig["supportY"].getDefault<double>(false);
    
    qLArm0[0] = walkConfig["qLArm"][1].getDefault<double>(false);
    qLArm0[1] = walkConfig["qLArm"][2].getDefault<double>(false);
    qLArm0[2] = walkConfig["qLArm"][3].getDefault<double>(false);
    
    qRArm0[0] = walkConfig["qRArm"][1].getDefault<double>(false);
    qRArm0[1] = walkConfig["qRArm"][2].getDefault<double>(false);
    qRArm0[2] = walkConfig["qRArm"][3].getDefault<double>(false);

    hardnessSupport = walkConfig["hardnessSupport"].getDefault<double>(false);
    hardnessSwing = walkConfig["hardnessSwing"].getDefault<double>(false);

    tStep0 = walkConfig["tStep"].getDefault<double>(false);
    tStep = walkConfig["tStep"].getDefault<double>(false);
    tZmp = walkConfig["tZmp"].getDefault<double>(false);
    stepHeight0 = walkConfig["stepHeight0"].getDefault<double>(false);
    stepHeight = walkConfig["stepHeight"].getDefault<double>(false);

    ph1Single = walkConfig["phSingle"][1].getDefault<double>(false);
    ph2Single = walkConfig["phSingle"][2].getDefault<double>(false);
    
    hipRollCompensation = walkConfig["hipRollCompensation"].getDefault<double>(false);

    ankleMod[0] = walkConfig["ankleMod"][1].getDefault<double>(false);
    ankleMod[1] = walkConfig["ankleMod"][2].getDefault<double>(false);

    velFastForward = walkConfig["velFastForward"].getDefault<double>(false);
    velFastTurn = walkConfig["velFastTurn"].getDefault<double>(false);
    supportFront = walkConfig["supportFront"].getDefault<double>(false);
    supportFront2 = walkConfig["supportFront2"].getDefault<double>(false);
    supportBack = walkConfig["supportBack"].getDefault<double>(false);
    supportSideX = walkConfig["supportSideX"].getDefault<double>(false);
    supportSideY = walkConfig["supportSideY"].getDefault<double>(false);
    supportTurn = walkConfig["supportTurn"].getDefault<double>(false);
    frontComp = walkConfig["frontComp"].getDefault<double>(false);
    AccelComp = walkConfig["AccelComp"].getDefault<double>(false);
    supportModYInitial = walkConfig["supportModYInitial"].getDefault<double>(false);
    walkKickPh = walkConfig["walkKickPh"].getDefault<double>(false);
    turnCompThreshold = walkConfig["turnCompThreshold"].getDefault<double>(false);
    turnComp = walkConfig["turnComp"].getDefault<double>(false);
    
    ankleImuParamX[0] = walkConfig["ankleImuParamX"][1].getDefault<double>(false);
    ankleImuParamX[1] = walkConfig["ankleImuParamX"][2].getDefault<double>(false);
    ankleImuParamX[2] = walkConfig["ankleImuParamX"][3].getDefault<double>(false);
    ankleImuParamX[3] = walkConfig["ankleImuParamX"][4].getDefault<double>(false);
    
    ankleImuParamY[0] = walkConfig["ankleImuParamY"][1].getDefault<double>(false);
    ankleImuParamY[1] = walkConfig["ankleImuParamY"][2].getDefault<double>(false);
    ankleImuParamY[2] = walkConfig["ankleImuParamY"][3].getDefault<double>(false);
    ankleImuParamY[3] = walkConfig["ankleImuParamY"][4].getDefault<double>(false);
    
    kneeImuParamX[0] = walkConfig["kneeImuParamX"][1].getDefault<double>(false);
    kneeImuParamX[1] = walkConfig["kneeImuParamX"][2].getDefault<double>(false);
    kneeImuParamX[2] = walkConfig["kneeImuParamX"][3].getDefault<double>(false);
    kneeImuParamX[3] = walkConfig["kneeImuParamX"][4].getDefault<double>(false);
    
    hipImuParamY[0] = walkConfig["hipImuParamY"][1].getDefault<double>(false);
    hipImuParamY[1] = walkConfig["hipImuParamY"][2].getDefault<double>(false);
    hipImuParamY[2] = walkConfig["hipImuParamY"][3].getDefault<double>(false);
    hipImuParamY[3] = walkConfig["hipImuParamY"][4].getDefault<double>(false);
    
    armImuParamX[0] = walkConfig["armImuParamX"][1].getDefault<double>(false);
    armImuParamX[1] = walkConfig["armImuParamX"][2].getDefault<double>(false);
    armImuParamX[2] = walkConfig["armImuParamX"][3].getDefault<double>(false);
    armImuParamX[3] = walkConfig["armImuParamX"][4].getDefault<double>(false);
    
    armImuParamY[0] = walkConfig["armImuParamY"][1].getDefault<double>(false);
    armImuParamY[1] = walkConfig["armImuParamY"][2].getDefault<double>(false);
    armImuParamY[2] = walkConfig["armImuParamY"][3].getDefault<double>(false);
    armImuParamY[3] = walkConfig["armImuParamY"][4].getDefault<double>(false);

    /////////////////////////////////////////////-----Initialization-----/////////////////////////////////////////////

    kickDefLeft.resize(7);
    
    kickDefLeft[0].resize(4);
    kickDefLeft[0][0].resize(1);
    kickDefLeft[0][1].resize(1);
    kickDefLeft[0][2].resize(3);
    kickDefLeft[0][3].resize(1);

    kickDefLeft[1].resize(6);
    kickDefLeft[1][0].resize(1);
    kickDefLeft[1][1].resize(1);
    kickDefLeft[1][2].resize(3);
    kickDefLeft[1][3].resize(3);
    kickDefLeft[1][4].resize(1);
    kickDefLeft[1][5].resize(1);
    
    kickDefLeft[2].resize(7);
    kickDefLeft[2][0].resize(1);
    kickDefLeft[2][1].resize(1);
    kickDefLeft[2][2].resize(3);
    kickDefLeft[2][3].resize(3);
    kickDefLeft[2][4].resize(1);
    kickDefLeft[2][5].resize(1);
    kickDefLeft[2][6].resize(1);

    kickDefLeft[3].resize(7);
    kickDefLeft[3][0].resize(1);
    kickDefLeft[3][1].resize(1);
    kickDefLeft[3][2].resize(3);
    kickDefLeft[3][3].resize(3);
    kickDefLeft[3][4].resize(1);
    kickDefLeft[3][5].resize(1);
    kickDefLeft[3][6].resize(1);

    kickDefLeft[4].resize(7);
    kickDefLeft[4][0].resize(1);
    kickDefLeft[4][1].resize(1);
    kickDefLeft[4][2].resize(3);
    kickDefLeft[4][3].resize(3);
    kickDefLeft[4][4].resize(1);
    kickDefLeft[4][5].resize(1);
    kickDefLeft[4][6].resize(1);

    kickDefLeft[5].resize(3);
    kickDefLeft[5][0].resize(1);
    kickDefLeft[5][1].resize(1);
    kickDefLeft[5][2].resize(3);

    kickDefLeft[6].resize(4);
    kickDefLeft[6][0].resize(1);
    kickDefLeft[6][1].resize(1);
    kickDefLeft[6][2].resize(3);
    kickDefLeft[6][3].resize(1);

    LuaTable kickConfig (LuaTable::fromFile("Config_Kick.lua"));
    for (int i = 0; i < int(kickDefLeft.size()); i++) {
        for (int j = 0; j < int(kickDefLeft[i].size()); j++){
            for (int k = 0; k < int(kickDefLeft[i][j].size()); k++) {
                if (kickDefLeft[i][j].size() == 1) {
                    kickDefLeft[i][j][k] = kickConfig["def"]["kickForwardLeft"]["def"][i+1][j+1].getDefault<double>(false);
                } else if (kickDefLeft[i][j].size() > 1) {
                    kickDefLeft[i][j][k] = kickConfig["def"]["kickForwardLeft"]["def"][i+1][j+1][k+1].getDefault<double>(false);
                }
                // std::cout<<i<<" "<<j<<" "<<k<<" "<<kickDefLeft[i][j][k]<<std::endl;
            }

        }
    }

    supportLeg = kickConfig["def"]["kickForwardLeft"]["supportLeg"].getDefault<double>(false);

    kickState = 1;
    uBody1 = {0, 0, 0};

    uTorso = {supportX, 0, 0};
    uLeft = {0, footY, 0};
    uRight = {0, -footY, 0};

    pLLeg = {0, footY, 0, 0, 0, 0};
    pRLeg = {0, -footY, 0, 0, 0, 0};
    pTorso = {supportX, 0, bodyHeight, 0, bodyTilt, 0};

    velCurrent = {0, 0, 0};
    velCommand = {0, 0, 0};
    velDiff = {0, 0, 0};

    aXP = 0;
    aXN = 0;
    aYP = 0;
    aYN = 0;

    kneeShift = 0;

    active = false;
    started = false;

    iStep0 = -1;
    iStep = 0;
    t0 = getTime();

    ph1Zmp = ph1Single;
    ph2Zmp = ph2Single;

    toeTipCompensation = 0;

    ph0 = 0;
    ph = 0;

    stopRequest = 2;
    walkKickRequest = 0;
    current_step_type = 0;
    initial_step = 2;
    upper_body_overridden = 0;
    motion_playing = 0;

    qLArmOR0[0] = qLArm0[0];
    qLArmOR0[1] = qLArm0[1];
    qLArmOR0[2] = qLArm0[2];
    
    qRArmOR0[0] = qRArm0[0];
    qRArmOR0[1] = qRArm0[1];
    qRArmOR0[2] = qRArm0[2];

    qLArmOR[0] = qLArm0[0];
    qLArmOR[1] = qLArm0[1];
    qLArmOR[2] = qLArm0[2];

    qRArmOR[0] = qRArm0[0];
    qRArmOR[1] = qRArm0[1];
    qRArmOR[2] = qRArm0[2];

    bodyRot = {0, bodyTilt, 0};

    phSingle = 0;

    uLRFootOffset = {0, footY + supportY, 0};

    uLeftI = {0, 0, 0};
    uRightI = {0, 0, 0};
    uTorsoI = {0, 0, 0};
    supportI = 0;
    start_from_step = false;
    stepkick_ready = false;
    has_ball = 0;

    for (int i = 0; i < NMOTORS; i++) {
        moveDir[i] = 1;
    }

    jointReverse = {
        0, // Head: 1,2
        // LArm: 3,4,5
        6,7,8, // LLeg: 6,7,8,9,10,11,
        15, // RLeg: 12,13,14,15,16,17
        17,19 // RArm: 18,19,20
    };

    for (int i = 0; i < int(jointReverse.size()); i++) {
        moveDir[jointReverse[i]] = -1;
    }
    /////////////////////////////////////////////-----Initialization-----/////////////////////////////////////////////
}

int main(int argc, char *argv[]) {
	////////////////////////////////////////Motors////////////////////////////////////////
    for (int i = 0; i < NMOTORS; i++) {
        moveDir[i] = 1;
    }

    jointReverse = {
        0, // Head: 1,2
        // LArm: 3,4,5
        6,7,8, // LLeg: 6,7,8,9,10,11,
        15, // RLeg: 12,13,14,15,16,17
        17,19 // RArm: 18,19,20
    };

    for (int i = 0; i < int(jointReverse.size()); i++) {
        moveDir[jointReverse[i]] = -1;
    }
    ////////////////////////////////////////Motors////////////////////////////////////////
	GOOGLE_PROTOBUF_VERIFY_VERSION;

	sprintf(host, "127.0.0.1");
	if (argc > 1) {
		if (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0) {
			printf("Usage: client <IP:port> or <IP> or <port>\n");
			return 0;
		}
		const char *n = strchr(argv[1], ':');
		if (n > 0) {
			port = atoi(n + 1);
			strncpy(host, argv[1], sizeof(host) - 1);
			host[n - argv[1]] = '\0';
		} else if (strchr(argv[1], '.') || !isdigit(argv[1][0]))
			strncpy(host, argv[1], sizeof(host) - 1);
		else
			port = atoi(argv[1]);
	}
	#ifdef _WIN32
		WSADATA info;
		rc = WSAStartup(MAKEWORD(2, 2), &info);  // Winsock 2.2
		if (rc != 0) {
			fprintf(stderr, "Cannot initialize Winsock\n");
			return 1;
		}
	#endif
	fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd == -1) {
		fprintf(stderr, "Cannot create socket\n");
		return 1;
	}
	memset(&address, 0, sizeof(struct sockaddr_in));
	address.sin_family = AF_INET;
	address.sin_port = htons(port);
	server = gethostbyname(host);

	if (server)
		memcpy((char *)&address.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
	else {
		fprintf(stderr, "Cannot resolve server name: %s\n", host);
		close_socket(fd);
		return 1;
	}
	rc = connect(fd, (struct sockaddr *)&address, sizeof(struct sockaddr));
	if (rc == -1) {
		fprintf(stderr, "Cannot connect to %s:%d\n", host, port);
		close_socket(fd);
		return 1;
	}
	char answer[8];
	n = recv(fd, answer, sizeof(answer), 0);
	if (n > 0) {
		if (strncmp(answer, "Welcome", 8) != 0) {
			if (strncmp(answer, "Refused", 8) == 0)
				printf("Connection to %s:%d refused: your IP address is not allowed in the game.json configuration file.\n", host,
					port);
			else
				printf("Received unknown answer from server: %s\n", answer);
			return 1;
		}
	} else {
		printf("Connection closed.\n");
		return 1;
	}
	printf("Connected to %s:%d\n", host, port);

	InitializeVariablesFromConfig();
	entry();
	for (;;) {
		if (!done) {
            update();
		}
	}

	close_socket(fd);
  	printf("Connection closed.\n");
  	return 0;
}
