# STM32 dataloggers example 

In this repository : 

* C code examples on how to collect data from current sensor, print the values through uart and then extend an embedded application with NEAI library



## Contents

- [NanoEdgeAI - Datalogging](#nanoedgeai-dataogging)
    - [Installation](#installation)
    - [Hardware](#hardware)
      - [Development boards](#development-boards)
      - [Sensors](#sensors)
      - [Wiring](#wiring)
    - [Software](#software)
      - [Prerequisites](#prerequisites)
    - [NanoEdge AI library integration](#nanoedge-ai-library-integration)
    - [Running and flashing the application on the board](#running-and-flashing-the-application-on-the-board)



## Installation

Clone NEAI-LAB repository.

## Hardware

These code examples allow to quickly extend existing data logger app with NEAI Studio based inferencing (n-Class classification) feature using following hardware:

### Development boards

- NUCLEO-U575ZI-Q :https://www.st.com/en/evaluation-tools/nucleo-u575zi-q.html
- X-NUCLEO-IKA01A1 : https://www.st.com/en/ecosystems/x-nucleo-ika01a1.html

## Wiring

Extend NUCLEO-U575ZI-Q with sensor shield assembling X-NUCLEO-IKA01A1 on the top of MCU board using Arduino connectors.


### Sensors

- ANALOG sensor (part of X-NUCLEO-IKA01A1)
- Current sensor : https://www.st.com/en/ecosystems/x-nucleo-ika01a1.html


## Software

### Prerequisites

These STM32Cube projects has been developped and tested under STM32CubeIDE v1.10.1. It requires installing following softwares and packages:

- The STM32CubeIDE software: [Download](https://www.st.com/en/development-tools/stm32cubeide.html#get-software)
- A serial port terminal, for example PuTTY: [Download](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html)

[optional] To create your own NanoEdge AI project, following softwares are also required:

- The NanoEdge AI Studio software: [Download](https://www.st.com/en/development-tools/nanoedgeaistudio.html#get-software)

  

### NanoEdge AI library integration

The NanoEdgeAI library is composed of 3 files to include in the STM32Cube project:

- The libneai.a file, to place in NEAI_Lib/Lib folder.
- The NanoEdgeAI.h header file, to place in the NEAI_Lib/Inc folder.
- The knowledge.h header file, to place in the NEAI_Lib/Inc folder.

To add the library and header files to the STM32VubeIDE environment perform below steps:

 Project -> Properties -> C/C++ General -> Paths and Symbols -> Libraries -> Add... -> neai
 Project -> Properties -> C/C++ General -> Path and Symbols -> Library Paths -> Add... -> NEAI_Lib/Lib
 Project -> Properties -> C/C++ General -> Paths and Symbols -> Includes -> Add... -> NEAI_Lib/Inc


### Running and flashing the application on the board

Connect your development board (by the STLK USB port) to your computer. Compile and flash the code using STM32CubeIDE. Use console application to see the output: 115200,8,N,1.

