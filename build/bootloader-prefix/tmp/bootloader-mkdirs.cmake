# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Program/Espressif/frameworks/esp-idf-v5.0.1/components/bootloader/subproject"
  "D:/Caesar/CodeArea/ESP32_Experiment/FloorLamp/build/bootloader"
  "D:/Caesar/CodeArea/ESP32_Experiment/FloorLamp/build/bootloader-prefix"
  "D:/Caesar/CodeArea/ESP32_Experiment/FloorLamp/build/bootloader-prefix/tmp"
  "D:/Caesar/CodeArea/ESP32_Experiment/FloorLamp/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Caesar/CodeArea/ESP32_Experiment/FloorLamp/build/bootloader-prefix/src"
  "D:/Caesar/CodeArea/ESP32_Experiment/FloorLamp/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Caesar/CodeArea/ESP32_Experiment/FloorLamp/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Caesar/CodeArea/ESP32_Experiment/FloorLamp/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
