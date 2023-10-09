# "Digital Voice Recorder"

## Objective:
The primary aim of this project is to develop a digital voice recorder capable of recording, playing back selected voice recordings, and managing recording data. Throughout the project, several modules, including Timer, PWM, ADC, and External Interrupts, will be employed.

## Methodology:
The system was coded in C using the STM32 Nucleo G031K8 board, with no reliance on HAL or equivalent libraries. The recording process involved an analog microphone with an onboard amplifier. To store up to five records, two 24LC512 Electrically Erasable Programmable Read-Only Memories (EEPROM) utilizing the I2C (Inter-Integrated Circuit) protocol were employed. Audio playback was facilitated through a speaker circuit with variable pot, LM386 amplifier, and various capacitors. A keypad was integrated for device operation, and a 7-segment display (7SD) was utilized to provide visual feedback on operations and status.

## Results:
The outcome of this project is a user-friendly digital voice recorder. It features a 7-segment display that provides memory status information and the capability to record up to four tracks, each lasting 5 seconds. A dedicated keypad button initiates voice recording, which automatically stops and saves the recording after a 5-second countdown displayed on the 7SD. Additional keypad buttons enable track selection, playback, pausing, deletion of selected tracks, and viewing track status, which is displayed on the 7SD when the corresponding button is pressed.

## Challenges:
Several challenges were encountered during this project. These included configuring PWM and ADC functionalities, which required significant troubleshooting. Ensuring clear and comprehensible audio recordings, addressing display flickering, and managing brightness differences on the 7-segment displays were also notable challenges that were overcome.

## Parts List:
- NUCLEO-G031K8
- JUMPER CABLE (X30)
- RESİSTANCE 1 kΩ (X9)
- 4xSEVEN SEGMENT (X1)
- 4x4KEYPAD
- TRANSİSTOR (X4)
- BREADBOARD (X1)
- 24LC512 I2C EEPROM (X2)
- SPEAKER (X1)
- LM386 LOWER POWER AMP (X1)
- ANALOG MICROPHONE (X1)
- Pot 100K (X1)
- CAPACITORS 1uF 220uF (X1)

## Schematic:
![Setup](https://github.com/MoaRajj/Digital_Voice_Recorder/assets/93192572/8dada204-f120-417e-b8de-bd6992010031)

## Flow Chart:
![FlowChart](https://github.com/MoaRajj/Digital_Voice_Recorder/assets/93192572/2f35aa73-0262-4bef-ba1f-7f954c6fc445)

## Block Diagram:
![BlockDiagram](https://github.com/MoaRajj/Digital_Voice_Recorder/assets/93192572/ade3f807-0883-4b4b-8e7f-2faad14eba53)

## State Transition Diagram:
![State transition diagram](https://github.com/MoaRajj/Digital_Voice_Recorder/assets/93192572/6099e445-e19e-4d1e-bf65-c85175eb99f7)

## Setup:
![WhatsApp Image 2021-01-14 at 8 22 11 PM](https://github.com/MoaRajj/Digital_Voice_Recorder/assets/93192572/3cf51e59-75b7-4878-b4df-fbfc68fa9df8)
![WhatsApp Image 2021-01-14 at 8 22 10 PM](https://github.com/MoaRajj/Digital_Voice_Recorder/assets/93192572/4669c997-6665-452e-a036-c69a743b5f0c)

