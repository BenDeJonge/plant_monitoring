# plant_monitoring

This repository houses the necessary code to perform advanced environmental monitoring and associated actuation for *sensitive house plants*, typically from Caribbean origin. The following **parameters** will be tracked:

Parameter                               | Symbol | Unit
----------------------------------------|--------|--------------
Air temperature                         | $T_a$  | $C$
Relative air humimidity                 | $RH_a$ | $\%$
Carbon dioxide ($CO_ 2$) content        | $pCO2$ | $\text{ppm}$
Other volatile organic compound content | $pVOC$ | $\text{ppm}$
Soil humidity                           | $RH_s$ | $\%$
Soil temperature                        | $T_s$  | $C$

Based on the above sensor readings, the following **actuations** will occur:

Parameter | Value | Actuation
----------|-------|-----------------------
$T_a$     | Low   | Heating element on
|         | High  | Heating element off
$RH_a$    | Low   | Ventilation off
|         | High  | Ventilation on
$RH_s$    | Low   | Watering of the plant

## Hardware

### Part list

1. Boards
    1. Arduino Uno Rev.3 (ATmega328P) with built-in WiFi module (ESP8266) 
2. Connections
    1. Breadboard 400 tie-point
    2. Three-pin cable coil ($10 m$)
    3. Jumper wires
    4. Relay $5 V$ JQC-3FF-S-Z for each actuator
    5. Flexible tubing ($1 m$)
3. Sensors
    1. ENS160 + AHT21 air quality sensor ($T_a, RH_a, pCO2, pVOC$)
    2. NE555 21K capacitative soil humidity sensor ($T_s, RH_s$)
4. Actuators
    1. Heating element
    2. Ventilation
    3. Submersible water pump $5 V$ JT-SL 80
5. Data visualization
    1. LCD with I2C interface (4 x 20 characters)
    2. Free Wix webserver (up to $500 MB$)

### Wiring diagram

TBD

### Pictures

TBD

## Software

TBD

### Embedded Arduino code

TBD

### Python webserver

TBD