
//------------------------------------------------------------------------------
// I M P O R T S
//------------------------------------------------------------------------------

#include <DFRobot_ENS160.h>
#include <Sensor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

//------------------------------------------------------------------------------
// G L O B A L S
//------------------------------------------------------------------------------

// Defining output pins
const int pinOutputSoil = 2;
const int pinOutputWatering = 3;
const int pinOutputAir = 4;
const int pinOutputLight = 5;

// Heater of ENS160 and thermistor of AHT21 on same PCB
// Causes major fluctuations in temperature readings
// Cannot measure continuously. Put in idle mode inbetween readings.

// Defining input pins
const int pinInputSoil = A0;
const int pinInputLight = A1;

// Timings
const int baudrate = 19200;
const unsigned long durationMeasurement = 10000;  // 10 s
// const unsigned long uploadTime = 3600000; // 1 hour
const unsigned long intervalLcd = 500;
const unsigned long intervalMeasurement = 60000;  // 2 mins

// Chronos
unsigned long previousTimeMeasurement, previousTimeLcd, previousTimeWatering, previousTime;
unsigned long cycles;
const int cycleThreshold = 2500;

// Toggles
bool firstRun, measuringSoil, watering, wateredThisCycle;
// bool heating = false;

// Sensour readouts
float soilMoisture;
float soilMoistureMean;
// Data containers
// Count cycles. Add values. Divide by number of cycles to grab mean.

// Sensor parameters
const float slopeSoilMoisture = 1.0;
const float interceptSoilMoisture = 0.0;
const float thresholdSoilMoisture = 2.0;  // L/kg
// Actuator settings
const unsigned long durationWatering = 15000;  // 15 s


//------------------------------------------------------------------------------
// S E T U P
//------------------------------------------------------------------------------

void setup() {
    // Setting output pins and instantiating as low
    int pinsOutput [] = {pinOutputSoil, pinOutputWatering, pinOutputAir, pinOutputLight};
    for (int pin: pinsOutput) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    // Setting input pins
    int pinsInput [] = {pinInputSoil, pinInputLight};
    for (int pin: pinsInput) {
        pinMode(pin, INPUT);
    }

    // Initializations
    Serial.begin(baudrate);
    lcd.init();
    lcd.backlight();

    // Chronos
    previousTimeMeasurement = 0;
    previousTimeLcd = 0;
    previousTimeWatering = 0;
    cycles = 0;

    // Toggles
    firstRun - true;
    measuringSoil = true;
    watering = false;
    wateredThisCycle = false;

    // Sensor readouts
    soilMoisture = 0.0;
    soilMoistureMean = 0.0;
}


//------------------------------------------------------------------------------
// M A I N   L O O P
//------------------------------------------------------------------------------

void loop() {
    
    // Check if it is time to do a measurement yet. Neglect on the first run.
    if ((millis() - previousTimeMeasurement >= intervalMeasurement) || (firstRun == true)) {
        // Serial.print("Starting soil measurement\n");
        previousTimeMeasurement = millis();
        firstRun = false;
        // Resetting the measurement.
        measuringSoil = true;
        soilMoisture = 0.0;
        cycles = 0;
        // Resetting the water states.
        watering = false;
        wateredThisCycle = false;
    }

    // Soil moisture measurement
    if (measuringSoil == true) {
        measuringSoil = toggleForTime(pinOutputSoil, previousTimeMeasurement, durationMeasurement, 0);
        cycles++;
        // Neglecting first measurements to charge capacitor
        if (cycles >= cycleThreshold) {
            soilMoisture += linRegSensor(mapSensor(pinInputSoil, 3),
                                        slopeSoilMoisture,
                                        interceptSoilMoisture);
        }
    // Measurement done. Compute mean
    } else {
        soilMoistureMean = soilMoisture / (cycles - cycleThreshold);
        // Serial.print("Measured mean soil moisture: ");
        // Serial.print(soilMoistureMean);
        // Serial.print(" mL/kg over ");
        // Serial.print(cycles - cycleThreshold);
        // Serial.print(" cycles\n");
        // Toggle watering on if it is not yet on
        if ((soilMoistureMean >= thresholdSoilMoisture) && (watering == false)) {
            watering = true;
            previousTimeWatering = millis();
            // if (wateredThisCycle == false) {
            //     Serial.print("WATERING ON\n");
            // } else {
            //     Serial.print("WATERING OFF\n");
            // }
        }
    }

    // Trigger a watering event of fixed duration
    if ((measuringSoil == false) && (watering == true) && (wateredThisCycle == false)) {
        // Serial.print("Watering calls toggleForTime ");
        watering = toggleForTime(pinOutputWatering, previousTimeWatering, durationWatering, 0);
        // Reset the trigger if the watering event is over
        if (watering == false) {
            wateredThisCycle = true;
        }
    }

    // Temperature measurement
    // Toggle heater

    // Carbon dioxide measurement
    // Toggle ventilation

    // Other VOCs measurements
    // Toggle warning LED


    // If temperature is higher, compute delta and decide heater time based on that.
    // Similar for pCO2.
    unsigned long lampTime = getLampTime(pinInputLight);
    // Display current data to LCD after the measurement is done.
    if ( (millis() - previousTimeLcd >= intervalLcd)) {
        writeLcdMessage(soilMoistureMean, 22.459, 89.685, 658.874, lampTime);
        previousTimeLcd = millis();
        }
    // Create upload cycle loop
    // Simply replace by checking if data arrays are full? Upload when e.g. n=60 measurements have been taken +- 1 hour.
    // In Python webserver: get last timepoint of last update. Compute timedelta. Subdivide in n equal parts. Associate each measurement with its own timestamp.
    // Problem: not all measurements will have same delta. Soil every minute, air every 10 minutes.
    // DatetimeIndex + set_index + reindex + interpolate: https://stackoverflow.com/questions/38754132/pandas-interpolate-dataframe-with-new-length
    // Add lamp on/off bool to most frequent dataframe (soilMoisture)


}


void writeLcdMessage(float soilMoistureMean, float airTemp, float airMoisture, float ppmCo2, unsigned long lampTime) {
    // Print the following message to the LCD:
    // GND 1.3Lkg 7.1pH 102     soilMoisture    soilPh      nextSoilMeas
    // AIR 99.8%  27.3C 102     airMoisture     airTemp     nextAirMeas
    // PPM 1023CO2  1011VOC     ppmCo2          ppmVoc
    // LMP   01h30  >08h30<     lmpTimeOn       lmpTimeOff

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("HS ");
    lcd.print(soilMoistureMean, 1);
    lcd.print("L/kg   T ");
    lcd.print(airTemp, 1);
    lcd.setCursor(0, 1);
    lcd.print("CO2 ");
    if (ppmCo2 <= 1000) {
        lcd.print(" ");
    }
    lcd.print(ppmCo2, 0);
    lcd.print("ppm  HA ");
    lcd.print(airMoisture, 0);
    lcd.print("%");

    int quarters = lampTime / 15 / 60 / 1000;
    int displayHours = quarters / 4;
    int displayMins = 15 * (quarters % 4);
    Serial.print(displayHours);
    Serial.print(":");
    Serial.println(displayMins);
    lcd.setCursor(0, 3);
    lcd.print("LMP  ");
    if (displayHours < 10) {
        lcd.print("0");
    }
    lcd.print(displayHours);
    lcd.print(":");
    if (displayMins < 10) {
        lcd.print("0");
    }
    lcd.print(displayMins);
}


unsigned long getLampTime(int pinPotentiometer) {
    // Map the potentiometer voltage to an integer number of quarters. Return as millis.
    float outputVoltage = mapSensor(pinPotentiometer, 5);
    // Serial.println("V: ");
    // Serial.print(outputVoltage);
    float quarters = outputVoltage * (96.0 / 5.0);
    // Serial.print(" quarters: ");
    // Serial.print(quarters);
    unsigned long lampTime = round(quarters) * 15 * 60 * 1000;
    // Serial.print(" millis: ");
    return lampTime;
}