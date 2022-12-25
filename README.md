![图片3](https://user-images.githubusercontent.com/113784775/209031555-ec5e717f-bddb-4b41-bc15-5e98aa44d442.png)

# Table of contents
1. [Project Overview](#1)
2. [Instruction](#2)
3. [Project Development](#3)
4. [Troubleshooting](#4)
5. [Reflection](#5)
6. [Satisfying Accomplishment](#6)
7. [PIO Part of Code](#7)
8. [Team Member](#8)

# Project Overview <a name="1"></a>

House safeguard system is an effective measure to solve the security management at the entrance and exit of important departments. Applicable to various confidential departments, such as banks, hotels, parking lot management, computer rooms, factories, etc.

This system equipped with a camera can capture the image of the visitor and remind the user the identification of the intruder coming in the building.
Also, users can realize bidirectional monitoring and control to the accees of their houses. More importantly, the camera switching would be decided by proximity of visitors, this feature developed the low-power mode for the safeguard system. 

![9468d6a5488b16e94c7766e25f1e825](https://user-images.githubusercontent.com/113784775/209005894-b024bc01-262d-42ec-a37d-ac682358e78c.jpg)

# Instruction <a name="2"></a>

## Components

### Pico4ml

![205468396-f7646e87-75a1-4214-ab9a-8d6017183e20](https://user-images.githubusercontent.com/113784775/209045372-c8843e79-387b-4556-8c21-ccf0b099a38a.jpg)

Amazon: https://www.amazon.com/Arducam-Pico4ML-TinyML-RP2040-Onboard/dp/B0919KQ7NV

### HC-05 Bluetooth

![image](https://user-images.githubusercontent.com/113784775/209006518-f7730e44-179e-439b-9c0a-ea14ad34d1f7.png)

Amazon: https://www.amazon.com/HiLetgo-Wireless-Bluetooth-Transceiver-Arduino/dp/B071YJG8DR

### VGA Cable

![image](https://user-images.githubusercontent.com/113784775/209006745-9724eaf8-04a6-460d-82e7-4cf86547dd45.png)

Amazon: https://www.amazon.com/Pasow-Monitor-Cable-Computer-Projector/dp/B08B64KM93

### Buzzer

![51zOdI55ndL _AC_SX679_](https://user-images.githubusercontent.com/113784775/209045442-5163b8b3-a057-4ecc-81ac-f0a3eb5c85ff.jpg)

Amazon: https://www.amazon.com/Cylewet-Electronic-Magnetic-Continuous-Arduino/dp/B01N7NHSY6

### QT Py RP2040

![adafruit_qtpy_rp2040](https://user-images.githubusercontent.com/113784775/209045525-7c645e30-88a0-469b-9aac-a1c83823e42b.jpg)

Adafruit: https://www.adafruit.com/product/4900

### APDS 9960

![3595-03](https://user-images.githubusercontent.com/113784775/209008146-7e1b2b2c-2621-46ea-a47b-6b4efc9e97e6.jpg)

Adafruit: https://www.adafruit.com/product/3595

## Assembly

### Overview

![9468d6a5488b16e94c7766e25f1e825](https://user-images.githubusercontent.com/113784775/209008305-e5cfc9e0-2864-444b-94f8-6f5eeb9a1588.jpg)

Pico4ml is controlled by the A3 pin of QT Py RP2040. Remember that all devices need to connect to ground.

### VGA Connection

![209008429-0c7eedca-1f97-473b-bcb6-910f6b9d54e3](https://user-images.githubusercontent.com/113784775/209045639-7f407ce2-0f60-412a-86eb-131d919688f7.png)

![微信图片_20221221165025](https://user-images.githubusercontent.com/113784775/209008699-a0992c8d-1428-441f-8dc7-fb4a902d96fd.jpg)

Both of HSYNC and VSYNC are connected from the GPIO ports (17 and 21) of the RP2040 to the appropriate pins on the VGA connector. The display expects a voltage in the range of 0-0.7V. The output from the RP2040 is 3.3V. So, putting resistors between the R/G/B color pins (18, 19, and 20) and the VGA connector creates a voltage divider that keeps the output voltage within a safe range for the display.

### Bluetooth Connection

![image](https://user-images.githubusercontent.com/113784775/209009473-d8b71b0e-6a13-461b-84e7-e6d08e76d7d4.png)

![微信图片_20221221165834](https://user-images.githubusercontent.com/113784775/209010072-11a015bd-9995-4d9b-a1bb-20cd1bb28fd5.jpg)

HC-05 bluetooth uses the 2.45GHz frequency band. The transfer rate of the data can vary up to 1Mbps and is in range of 10 meters. The HC-05 module can be operated within 4-6V of power supply. It supports baud rate of 9600, 19200, 38400, 57600, etc. TXD of HC-05 connects to the A0 pin of QT Py RP2040, and RXD connects to the A1 pin.

### APDS 9960

APDS 9960 connects to QT Py RP2040 through STEMMA QT / Qwiic JST SH 4-pin Cable.

### Buzzer

Buzzer connects to the A3 pin of QT Py RP2040.

## Block Diagram

![屏幕截图 2022-12-21 171247](https://user-images.githubusercontent.com/113784775/209012161-063ebedd-2c7d-49bf-b55e-f523b14568d1.png)

RP2040 controls modules in the system. APDS9960 measures distance from the visitor. When the visitor is close to a certain distance, the buzzer alerts, and the camera in Pico4ML captures the image and sends to the screen. After checking the identification of the visitor, the user can change the status of the system (alarm on or off) through the Bluetooth.

## RP 2040 Set Up Environment

https://github.com/zgu74/Setup-Guide.git

## Project Code

[Project Code.zip](https://github.com/zgu74/zgu74.github.io/files/10289907/Project.Code.zip)


# Project Development <a name="3"></a>

We tried to develop our project step by step. We divided the project into three parts: the VGA function, the bluetooth function, and the combination. Since we had developed APDS 9960 in the previous labs, the distance detection was not listed as a major part. Because of online resources, we did not encounter too many difficulties when we set up the HC-05 bluetooth. There was also an bluetooth terminal App on Google Play Store so that we did not need to build an bluetooth terminal ourselves. The VGA part was the biggest challenge in the project. Thanks for the advice from Professor Banks. Troubleshooting will be discussed in the nest section. At last, we combined all components to achieve the function we designed in the proposal.

# Troubleshooting <a name="4"></a>

## VGA Input Timing

Initially the current input timing is not supported by the monitor display.

![微信图片_20221221195821](https://user-images.githubusercontent.com/113784775/209031907-995c6b43-55b7-42c3-a7f7-dd001fff2ba5.jpg)

We check the timing of Pico4ML output via the logic analyzer and change the output frequency to the timing supported by the screen. The first one is the correct frequency. The second one is half due to the clock of the camera.

![image](https://user-images.githubusercontent.com/113784775/209031936-1508ae3b-1416-4a6c-aeff-bce97014d670.png)

![image](https://user-images.githubusercontent.com/113784775/209031946-03bf2210-6876-4f71-8d63-3cbd4ec20757.png)

## VGA Display

We enlarge the image to the whole display by changing pixel. The first picture is the original display. The second one is the revised display.

![205467458-4bdabee5-d0f7-4fdd-b7a5-f152e3e446d3](https://user-images.githubusercontent.com/113784775/209033211-bf3e99d8-678d-4406-a6ca-9681d8241596.jpg)

![图片6](https://user-images.githubusercontent.com/113784775/209033369-3fd41644-c7cf-4cf4-a7ed-679028e4120c.jpg)

We also implemted the image processing algorithm. The camera on Pico4ml is a monochrome HiMax HM01B0 camera. Eight colors represent eight grayscale levels. The brightness of most pixels on face are concentrated in [32,96). We expand this interval by a coefficient of 2. Therefore, there are more colors on the face, less on background. The image so shows more clearly. We use “pixel=pixel/32” to set pixels as 8 colors.  

Compared with fuzzy and unclear images in baseline version, our optmized design is able to display face-recognizable image on the screen.

![屏幕截图 2022-12-21 202218](https://user-images.githubusercontent.com/113784775/209034186-0c902e39-b2d1-4f3e-ade4-a8fbec5843ff.png)

![图片4](https://user-images.githubusercontent.com/113784775/209034206-e4e14ee3-a6d5-485b-8543-f159870b4262.jpg)

## gpio_get()

At first, we could not use function “gpio_get()”. Then we tried to disable LCD code. The function could work correctly. We think there was not enough ROM or RAM to run the code since there were too many large arrays.

## DMA

# Reflection <a name="5"></a>

## Pico4ml

It is a good tiny machine learning board. However, its LCD is unused in the project and the camera is monochhrome. Hence, it is better to employ a polychrome camera module directly instead of Pico4ml.

## QT Py RP2040

QT Py RP2040 is a tiny board. It has limited pinouts, and needs to be soldered for the good connection with other components. We recommend to use Raspberry Pi Pico with pre-soldered headers and more pins.

## HC-05 Bluetooth

HC-05 Bluetooth functions well. It can connect to *Serial Blurtooth Terminal* App on Google Play Store. 

## VGA Cable

Remember that Pico4ml is connected to the raw VGA cable instead of the VGA adapter. The VGA adapter fails to recognize the input signal in the project.

# Satisfying Accomplishment <a name="6"></a>
Transmitting the image captured by the camera to the VGA display is a challenged task. We make a great effort to improve the image: from simple images (square, circle, etc.) generated by codes to inappropriate timing inputs from Pico4ml, to the small and vague camera-captured image, to the enlarged but obscure image, to the last legible images. We are very glad to achieve the goal in the end.

# PIO Part of Code <a name="7"></a>  
The PIO modules in the design were used for the I/O interface,including the HC-05 Bluetooth module, VGA Data Transfer, APDS 9960 Sensor, and the LCD display on the pico4ML.  

In PICO4ML code, we use hsync.pio, vsync.pio and rgb.pio to control the VGA output, among them, hsync.pio, vsync.pio decide how we put the pixels to the screen, for example from left to right or from bottom to top, rgb.pio decides which color we want to display. Then we use image.pio to process the camera data. And in QT py RP2040 code, we use i2c.pio to communicate with APDS9960 and read data from it.

![image](https://user-images.githubusercontent.com/113784775/209220211-34255812-c8e7-40f4-ab0b-958c8422822a.png)


# Team Member <a name="8"></a>

Chenwei Tang: https://github.com/Chenwei-Tang

Shuhan Qian: https://github.com/QSHANSSS

Zeyu Gu: https://github.com/zgu74




