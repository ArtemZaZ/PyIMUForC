cmake_minimum_required(VERSION 3.9)
project(PyIMUModuleForC C)

set(CMAKE_C_STANDARD 99)

add_executable(PyIMUModuleForC main.c IMUlib.h IMUlib.c PyIMU.c PyIMU.h)