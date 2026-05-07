# Fly-Tipping Monitoring and Notification System

This repository contains the final firmware used for the two-node fly-tipping monitoring prototype developed for the 6E6Z0012 Individual Project.

## System overview

The prototype uses two Seeed XIAO ESP32-S3 boards.

- Node 1: XIAO ESP32-S3 Sense with OV3660 camera and microSD card. This node handles PIR and ToF detection, camera capture, SD storage and UART trigger transmission.
- Node 2: XIAO ESP32-S3 with Wio-SX1262 LoRa radio. This node receives UART event triggers from Node 1 and transmits the LoRa payload.

## Firmware files

- `src/XIAO1_Detection_Camera_Node/XIAO1_Detection_Camera_Node.ino`
- `src/XIAO2_LoRa_Radio_Node/XIAO2_LoRa_Radio_Node.ino`

## Arduino IDE settings

Board: XIAO_ESP32S3  
PSRAM: OPI PSRAM  
USB CDC On Boot: Enabled  
Upload Mode: UART0 / Hardware CDC  
Upload Speed: 921600  

## Required libraries

- Pololu VL53L1X
- RadioLib by Jan Gromeš
- ESP32 Arduino core SD library
- ESP32 camera driver

## Notes

The firmware in this repository corresponds to the final evaluated prototype version described in the dissertation report.
