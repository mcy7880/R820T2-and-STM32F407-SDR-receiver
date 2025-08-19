# R820T2-and-STM32F407-SDR-receiver
It's experimental SDR (Software Defined Radio) receiver based on the STM32F407 and R820T2 (actually R820T). The repository contains source code for STM32F407 (complete project from STM32IDE) and hardware schematics. Frankly it's more fun and educational project rather than reliable SDR receiver. The motivation for this project was built SDR based on STM32F4DISCOVERY (it has CS43L22 audio DAC) board and R820T module that comes from obsolete DVB-T set-top-box.

R820T2 code is based on Eric's Brombaugh's code: https://github.com/emeb/r820t2/tree/master

Porting, modification, improvements and SDR code was done by @mcy7880 Maciej Fajfer.

# Doc
Documentation, hardware schematics and block diagram.

# Matlab
Matlab's script and *.FDA files for Filter Designer (fdatool).

# stm32f407_r820t2
STM32F407 - the whole project from STM32IDE

Lots of detials (in Polish): https://www.elektroda.pl/rtvforum/topic4133002.html

# Edit - 19-08-2025
There was some issues with IIR audio filer (stability) for AM demodulator and some minor issues with UART console. I uploaded new version of the source code.