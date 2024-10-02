#ifndef MMWAVERADAR_IMP_H
#define MMWAVERADAR_IMP_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <thread>
#include <mutex>

#define STATE_DIM 6
#define SPHER_DIM 4

#define MAX_BUFFER_SIZE 2048
#define HEADER_SIZE_IN_BYTES 40
#define MAX_BUFFERED_FRAMES_SIZE 10
#define MAX_BUFFERED_COMPLETE_DATA 10
#define MAX_DETECTED_OBJECTS 15

enum message_type_e {
    MSG_DETECTED_POINTS = 1,
    MSG_RANGE_PROFILE,
    MSG_NOISE_PROFILE,
    MSG_AZIMUTH_STATIC_HEAT_MAP,
    MSG_RANGE_DOPPLER_HEAT_MAP,
    MSG_STATS,
    MSG_DETECTED_POINTS_SIDE_INFO,
    MSG_AZIMUTH_ELEVATION_STATIC_HEAT_MAP,
    MSG_TEMPERATURE_STATS,
    MSG_MAX
};

typedef struct data_header_t {
    uint8_t magicBytes[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
    uint32_t version;
    uint32_t totalPacketLen;
    uint32_t platform;
    uint32_t frameNumber;
    uint32_t timeCpuCycles;
    uint32_t numDetectedObj;
    uint32_t numTLVs;
    uint32_t subFrameNumber;
} data_header_t;

typedef struct data_tl_t {
    uint32_t type;
    uint32_t length;
} data_tl_t;

typedef struct detected_object_t {
    float x, y, z;
    float velocity;
    Eigen::VectorXd stateVector = Eigen::VectorXd(STATE_DIM);
    Eigen::VectorXd spherVector = Eigen::VectorXd(SPHER_DIM);
} detected_object_t;

typedef struct data_complete_t {
    data_header_t dataHeader;
    data_tl_t dataTL;
    std::vector<detected_object_t> detectedObjects;
} data_complete_t;

extern data_header_t dataHeader;
extern data_tl_t dataTL;
extern detected_object_t detectedObject;
extern std::vector<detected_object_t> detectedObjects;
extern data_complete_t dataComplete;

extern std::mutex mtx;

float PackRGB(uint8_t r, uint8_t g, uint8_t b);

#endif