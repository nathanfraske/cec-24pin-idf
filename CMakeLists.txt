cmake_minimum_required(VERSION 3.16)

# Project name; this becomes the binary filename
set(PROJECT_NAME cec_24pin)

# Pull in ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(${PROJECT_NAME})
