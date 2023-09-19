# "Digital Voice Recorder"

## Objective:
The primary aim of this project is to develop a digital voice recorder capable of recording, playing back selected voice recordings, and managing recording data. Throughout the project, several modules, including Timer, PWM, ADC, and External Interrupts, will be employed.

## Methodology:
The system was coded in C using the STM32 Nucleo G031K8 board, with no reliance on HAL or equivalent libraries. The recording process involved an analog microphone with an onboard amplifier. To store up to five records, two 24LC512 Electrically Erasable Programmable Read-Only Memories (EEPROM) utilizing the I2C (Inter-Integrated Circuit) protocol were employed. Audio playback was facilitated through a speaker circuit with variable pot, LM386 amplifier, and various capacitors. A keypad was integrated for device operation, and a 7-segment display (7SD) was utilized to provide visual feedback on operations and status.

## Results:
The outcome of this project is a user-friendly digital voice recorder. It features a 7-segment display that provides memory status information and the capability to record up to four tracks, each lasting 5 seconds. A dedicated keypad button initiates voice recording, which automatically stops and saves the recording after a 5-second countdown displayed on the 7SD. Additional keypad buttons enable track selection, playback, pausing, deletion of selected tracks, and viewing track status, which is displayed on the 7SD when the corresponding button is pressed.

## Challenges:
Several challenges were encountered during this project. These included configuring PWM and ADC functionalities, which required significant troubleshooting. Ensuring clear and comprehensible audio recordings, addressing display flickering, and managing brightness differences on the 7-segment displays were also notable challenges that were overcome.

## Relevance:
This project strongly aligns with my interests in Embedded Systems, demanding in-depth knowledge of microcontrollers and EEPROM functionality to calculate memory data size and determine the appropriate data rate for meeting the recording period requirements.
