cmake_minimum_required(VERSION 3.10)
project(mmWave-RADAR-Team)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_policy(SET CMP0144 NEW)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set(LIBSERIAL_INCLUDE_DIR "/usr/local/include/libserial")
set(LIBSERIAL_LIBRARY "/usr/local/lib/libserial.so")
include_directories(${LIBSERIAL_INCLUDE_DIR})

add_executable(
    ${PROJECT_NAME} #EXE Name
    source/main.cpp
    source/mmWaveRadar.cpp
    source/mmWaveRadar_imp.cpp
)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${LIBSERIAL_LIBRARY})