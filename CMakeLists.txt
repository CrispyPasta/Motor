cmake_minimum_required(VERSION 3.10)
project(Motor)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_library(JetsonGPIO_LIBRARY NAMES JetsonGPIO HINTS "~/Downloads/JetsonGPIO/build" REQUIRED)
#include_directories(${JetsonGPIO_INCLUDE_DIRS})
#set(LIBS ${LIBS} ${JetsonGPIO_LIBRARIES})
#target_link_libraries(Motor ${LIBS})
#target_link_libraries(Motor Threads::Threads)

set(CMAKE_CXX_STANDARD 14)

add_executable(Motor main.cpp PID/PID.cpp PID/PID.h)
