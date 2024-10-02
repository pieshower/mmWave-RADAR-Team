#include <eigen3/Eigen/Dense>

#include "../include/mmWaveRadar.h"
#include "../include/iwr1843Config.h"

mmWaveRadar::mmWaveRadar() {
    // initKalmanVariables();
    connectPort();
}

void mmWaveRadar::configure(const char* configCommands[], const unsigned long configSize) {
    if (!userPort_error) {
        std::string command = configDataPort + "\r\n";
        userPort.Write(command);
        usleep(100000);
        command = sensorStop + "\r\n";
        userPort.Write(command);
        usleep(100000);
        for (unsigned long i = 0; i < configSize; i++) {
            command = std::string(configCommands[i]) + "\r\n";
            userPort.Write(command);
            usleep(10000);
        }
        std::cout << "mmWaveRadar should be configured..." << std::endl;
    }
    else {
        std::cerr << "User port is not open..." << std::endl;
    }
}

void mmWaveRadar::connectPort() {
    try {
        userPort.Open(userPort_s);
        userPort.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    } 
    catch (LibSerial::OpenFailed &e) {
        std::cerr << "unable to open user port" << std::endl;
        userPort_error = true;
    }

    try {
        dataPort.Open(dataPort_s);
        dataPort.SetBaudRate(LibSerial::BaudRate::BAUD_921600);
    } 
    catch (LibSerial::OpenFailed &e) {
        std::cerr << "unable to open data port" << std::endl;
        dataPort_error = true;
    }
}

void mmWaveRadar::start() {
    configure(iwr1843ConfigCommands, configCommandsSize);
    std::string command = sensorStart + "\r\n";
    userPort.Write(command);
    usleep(10000);
    std::cout << "Starting mmWaveRadar..." << std::endl;
}

void mmWaveRadar::stop() {
    std::string command = sensorStop + "\r\n";
    userPort.Write(command);
    usleep(10000);
    std::cout << "Stopping mmWaveRadar..." << std::endl;
}

void mmWaveRadar::read() {
    std::vector<uint8_t> buf;

    while (buf.size() < MAX_BUFFER_SIZE) {
        uint8_t byte;
        dataPort.ReadByte(byte);
        buf.push_back(byte);
    }
    
    parseFrames(buf, frame, frames);

    for (std::vector<uint8_t> &_frame : frames) {
        parseFrame(_frame);
    }
    frames.clear();
}

void mmWaveRadar::parseFrames(std::vector<uint8_t> &_buf, std::vector<uint8_t> &_frame, std::vector<std::vector<uint8_t>> &_frames) {
    const std::vector<uint8_t> magicBytes = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
    for (size_t i = 0; i <= _buf.size() - sizeof(data_header_t::magicBytes);) {
        if (std::equal(magicBytes.begin(), magicBytes.end(), _buf.begin() + i)) {
            size_t frameStart = i;
            uint32_t frameSize = (_buf[i + 15] << 24) | (_buf[i + 14] << 16) | (_buf[i + 13] << 8) | (_buf[i + 12]);
            if (i + frameSize <= _buf.size()) {
                _frame.insert(_frame.end(), _buf.begin() + frameStart, _buf.begin() + i + frameSize);
                _frames.push_back(_frame);
                _frame.clear();
                _buf.erase(_buf.begin(), _buf.begin() + i + frameSize);
                i = 0;
            } else {
                break;
            }
        } else {
            i++;
        }
    }
}

void mmWaveRadar::parseFrame(std::vector<uint8_t> &_frame) {
    // std::cout << std::endl << std::dec;
    parseFrameHeader(_frame, dataHeader);
    // std::cout << "current frame (" << _frame.size() << " size):" << std::endl;
    // std::cout << "Actual parsed packet length: " << dataHeader.totalPacketLen << std::endl;
    // std::cout << "current number of objects detected: " << dataHeader.numDetectedObj << std::endl;
    parseFrameTL(_frame, dataTL);
    // std::cout << "current number tlv length: " << dataTL.length << std::endl << std::endl;
    parseFrameDetectedObjects(_frame, detectedObject, detectedObjects);
    // updateDataComplete(dataComplete, dataHeader, dataTL, detectedObjects);
}

void mmWaveRadar::parseFrameHeader(std::vector<uint8_t> &_frame, data_header_t &_dataHeader) {
    uint8_t dataHeader_i = 0;
    for (size_t i = sizeof(data_header_t::magicBytes); i + 3 < _frame.size() && i < sizeof(data_header_t); i += 4) {
        uint32_t doubleword = (_frame[i + 3] << 24) | (_frame[i + 2] << 16) | (_frame[i + 1] << 8) | (_frame[i]);
        switch (dataHeader_i) {
            case 0: _dataHeader.version = doubleword; break;
            case 1: _dataHeader.totalPacketLen = doubleword; break;
            case 2: _dataHeader.platform = doubleword; break;
            case 3: _dataHeader.frameNumber = doubleword; break;
            case 4: _dataHeader.timeCpuCycles = doubleword; break;
            case 5: _dataHeader.numDetectedObj = doubleword; break;
            case 6: _dataHeader.numTLVs = doubleword; break;
            case 7: _dataHeader.subFrameNumber = doubleword; break;
        }
        dataHeader_i++;
    }
}

void mmWaveRadar::parseFrameTL(std::vector<uint8_t> &_frame, data_tl_t &_dataTL) {
    uint8_t dataTL_i = 0;
    for (size_t i = sizeof(data_header_t); i + 3 < _frame.size() && i < sizeof(data_tl_t) + sizeof(data_header_t); i += 4) {
        uint32_t doubleword = (_frame[i + 3] << 24) | (_frame[i + 2] << 16) | (_frame[i + 1] << 8) | (_frame[i]);
        switch (dataTL_i) {
            case 0: _dataTL.type = doubleword; break;
            case 1: _dataTL.length = doubleword; break;
        }
        dataTL_i++;
    }
}

void mmWaveRadar::parseFrameDetectedObjects(std::vector<uint8_t> &_frame, detected_object_t &_detectedObject, std::vector<detected_object_t> &_detectedObjects) {
    uint8_t detectedObject_i = 0;

    _detectedObjects.clear();

    if (dataTL.type == message_type_e::MSG_DETECTED_POINTS) {
        for (size_t i = sizeof(data_header_t) + sizeof(data_tl_t); i + 3 < _frame.size() && i < sizeof(data_header_t) + sizeof(data_tl_t) + dataTL.length; i += 4) {
            float temp;
            uint32_t doubleword = (_frame[i + 3] << 24) | (_frame[i + 2] << 16) | (_frame[i + 1] << 8) | (_frame[i]);
            std::memcpy(&temp, &doubleword, sizeof(float));
            switch (detectedObject_i) {
                case 0: _detectedObject.x = temp; break;
                case 1: _detectedObject.y = temp; break;
                case 2: _detectedObject.z = temp; break;
                case 3: _detectedObject.velocity = temp; break;
            }
            detectedObject_i++;
            if (detectedObject_i > 3) {
                convertToVector(_detectedObject);
                _detectedObjects.push_back(_detectedObject);
                detectedObject_i = 0;
            }
        }
    }
    else {
        std::cout << "No detected objects.." << std::endl;
        return;
    }

    while (_detectedObjects.size() > MAX_DETECTED_OBJECTS) {
        _detectedObjects.pop_back();
    }
}

void mmWaveRadar::updateDataComplete(data_complete_t &_dataComplete, data_header_t &_dataHeader, data_tl_t &_dataTL, std::vector<detected_object_t> &_detectedObjects) {
    _dataComplete.dataHeader = _dataHeader;
    _dataComplete.dataTL = _dataTL;
    _dataComplete.detectedObjects = _detectedObjects;

    int num = 1;
    // std::cout << "Frame Count: " << frameCount++ << std::endl;
    for (detected_object_t i : _dataComplete.detectedObjects) {
        std::cout << "Object " << num << ":" << std::endl;
        std::cout << "x: " << i.x << std::endl;
        std::cout << "y: " << i.y << std::endl;
        std::cout << "z: " << i.z << std::endl;
        std::cout << "velocity: " << i.velocity << std::endl << std::endl;
        num++;
    }
}

void mmWaveRadar::convertToVector(detected_object_t &_detectedObject) {
    float x = _detectedObject.x;
    float y = _detectedObject.y;
    float z = _detectedObject.z;
    float velocity = _detectedObject.velocity;

    float magnitude = sqrt(x * x + y * y + z * z);

    float vx = velocity * (x / magnitude);
    float vy = velocity * (y / magnitude);
    float vz = velocity * (z / magnitude);

    float rho = sqrt(x * x + y * y + z * z);
    float theta = atan(y / x);
    
    if (rho < 0.0001) { rho = 0.0001; }

    float phi = acos(z / rho);
    float rho_dot_p = (x * x + y * y + z * z) / rho;

    Eigen::VectorXd newStateVector = Eigen::VectorXd(STATE_DIM);
    newStateVector << x, y, z, vx, vy, vz;
    // newStateVector[0] = x;
    // newStateVector[1] = y;
    // newStateVector[2] = z;
    // newStateVector[3] = vx;
    // newStateVector[4] = vy;
    // newStateVector[5] = vz;
    _detectedObject.stateVector = newStateVector;

    Eigen::VectorXd newSpherVector = Eigen::VectorXd(SPHER_DIM);
    newSpherVector << rho, theta, phi, rho_dot_p;
    // newSpherVector[0] = rho;
    // newSpherVector[1] = theta;
    // newSpherVector[2] = phi;
    // newSpherVector[3] = rho_dot_p;
    _detectedObject.spherVector = newSpherVector;
}