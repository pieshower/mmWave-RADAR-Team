#include "../include/mmWaveRadar.h"

void radarLoop() {
    mmWaveRadar::getRadarGuy().start();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    while (true) {
        if (mtx.try_lock()) {
            mmWaveRadar::getRadarGuy().read();
            mtx.unlock();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main() {
    std::thread radar(radarLoop);
    radar.detach();

    while (true);

    return 0;
}
