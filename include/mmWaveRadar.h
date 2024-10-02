#ifndef mmWaveRadar_H
#define mmWaveRadar_H

#include <libserial/SerialPort.h>

#include "../include/mmWaveRadar_imp.h"

class mmWaveRadar {
private:
    LibSerial::SerialPort userPort;
    LibSerial::SerialPort dataPort;

    std::string userPort_s = "/dev/ttyACM0";
    std::string dataPort_s = "/dev/ttyACM1";
    
    bool userPort_error = false;
    bool dataPort_error = false;
    
    static mmWaveRadar RadarGuy;

     mmWaveRadar();
    ~mmWaveRadar() = default;
    
    void configure(const char* configCommands[], const unsigned long configSize);
    void connectPort();

    void parseFrames(std::vector<uint8_t> &_buf, std::vector<uint8_t> &_frame, std::vector<std::vector<uint8_t>> &_frames);
    void parseFrame(std::vector<uint8_t> &_frame);
    void parseFrameHeader(std::vector<uint8_t> &_frame, data_header_t &_dataHeader);
    void parseFrameTL(std::vector<uint8_t> &_frame, data_tl_t &_dataTL);
    void parseFrameDetectedObjects(std::vector<uint8_t> &_frame, detected_object_t &_detectedObject, std::vector<detected_object_t> &_detectedObjects);

    void updateDataComplete(data_complete_t &_dataComplete, data_header_t &_dataHeader, data_tl_t &_dataTL, std::vector<detected_object_t> &_detectedObjects);

    void convertToVector(detected_object_t &_detectedObject);

    std::vector<uint8_t> frame;
    std::vector<std::vector<uint8_t>> frames;

public:
    static mmWaveRadar& getRadarGuy() { return RadarGuy; }

    void start();
    void  stop();
    void read();
};

inline mmWaveRadar mmWaveRadar::RadarGuy;

#endif