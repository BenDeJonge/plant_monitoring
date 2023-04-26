
//------------------------------------------------------------------------------
// I M P O R T S
//------------------------------------------------------------------------------

#include <DFRobot_ENS160.h>
#include <AHT20.h>
#include <Sensor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
DFRobot_ENS160_I2C ENS160(&Wire, /*I2CAddr*/ 0x53);
AHT20 AHT20;

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

// Chronos

struct Chronos {
    // Watering event
    unsigned long soilMeasurement;
    unsigned long watering;
    unsigned long lcdSensors;
    // Lamp
    unsigned long lamp;
    unsigned long lcdLamp;
    unsigned long cycles;
} myChronos;

struct Durations {
    unsigned long soilMeasurement;
    unsigned long soilMeasurementLength;
    unsigned long watering;
    unsigned long lcdSensors;
    unsigned long lcdLamp;
    unsigned long cycles;
} myDurations;

struct Toggles{
    bool firstRun;
    bool soilMeasurement;
    bool watering;
    bool wateredThisCycle;     
} myToggles;

struct Calibrations {
    float slopeSoilMoisture;
    float interceptSoilMoisture;
    float thresholdSoilMoisture;
} myCalibs;


// Sensour readouts
float soilMoisture, soilMoistureMean;
// Data containers
// Count cycles. Add values. Divide by number of cycles to grab mean.


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
    // Toggle air sensor on.
    digitalWrite(pinOutputAir, HIGH);

    // Setting input pins
    int pinsInput [] = {pinInputSoil, pinInputLight};
    for (int pin: pinsInput) {
        pinMode(pin, INPUT);
    }

    // Initializations
    Serial.begin(baudrate);
    lcd.init();
    lcd.backlight();
    writeLcdStructure();
    ENS160.setPWRMode(ENS160_STANDARD_MODE);
    ENS160.setTempAndHum(/*temperature=*/25.0, /*humidity=*/50.0);
    
    // Chronos
    myChronos.soilMeasurement = 0;
    myChronos.watering = 0;
    myChronos.lcdSensors = 0;
    myChronos.lamp = 0;
    myChronos.lcdLamp = 0;
    myChronos.cycles = 0;

    // Durations
    myDurations.soilMeasurement = 60000;
    myDurations.soilMeasurementLength = 10000;
    myDurations.watering = 15000;
    myDurations.lcdSensors = 3000;
    myDurations.lcdLamp = 300;
    myDurations.cycles = 25;

    // Toggles
    myToggles.firstRun = true;
    myToggles.soilMeasurement = true;
    myToggles.watering = false;
    myToggles.wateredThisCycle = false; 

    // Calibrations
    myCalibs.slopeSoilMoisture = 1.0;
    myCalibs.interceptSoilMoisture = 0.0;
    myCalibs.thresholdSoilMoisture = 2.0;

    // Sensor readouts
    soilMoisture = 0.0;
    soilMoistureMean = 0.0;


}


//------------------------------------------------------------------------------
// M A I N   L O O P
//------------------------------------------------------------------------------

void loop() {
    
    // Check if it is time to do a measurement yet. Neglect on the first run.
    if ((millis() - myChronos.soilMeasurement >= myDurations.soilMeasurement) || (myToggles.firstRun == true)) {
        // Serial.print("Starting soil measurement\n");
        myChronos.soilMeasurement = millis();
        myToggles.firstRun = false;
        // Resetting the measurement.
        myToggles.soilMeasurement = true;
        soilMoisture = 0.0;
        myChronos.cycles = 0;
        // Resetting the water states.
        myToggles.watering = false;
        myToggles.wateredThisCycle = false;
    }

    // Soil moisture measurement
    if (myToggles.soilMeasurement == true) {
        myToggles.soilMeasurement = toggleForTime(pinOutputSoil, myChronos.soilMeasurement, myDurations.soilMeasurementLength, 0);
        myChronos.cycles++;
        // Neglecting first measurements to charge capacitor
        if (myChronos.cycles > myDurations.cycles) {
            soilMoisture += linRegSensor(mapSensor(pinInputSoil, 3),
                                        myCalibs.slopeSoilMoisture,
                                        myCalibs.interceptSoilMoisture);
        }
    // Measurement done. Compute mean
    } else {
        soilMoistureMean = soilMoisture / (myChronos.cycles - myDurations.cycles);
        // Serial.print("Measured mean soil moisture: ");
        // Serial.print(soilMoistureMean);
        // Serial.print(" mL/kg over ");
        // Serial.print(cycles - cycleThreshold);
        // Serial.print(" cycles\n");
        // Toggle watering on if it is not yet on
        if ((soilMoistureMean >= myCalibs.thresholdSoilMoisture) && (myToggles.watering == false)) {
            myToggles.watering = true;
            myChronos.watering = millis();
            // if (wateredThisCycle == false) {
            //     Serial.print("WATERING ON\n");
            // } else {
            //     Serial.print("WATERING OFF\n");
            // }
        }
    }

    // Trigger a watering event of fixed duration
    if ((myToggles.soilMeasurement == false) && (myToggles.watering == true) && (myToggles.wateredThisCycle == false)) {
        // Serial.print("Watering calls toggleForTime ");
        myToggles.watering = toggleForTime(pinOutputWatering, myChronos.watering, myDurations.watering, 0);
        // Reset the trigger if the watering event is over
        if (myToggles.watering == false) {
            myToggles.wateredThisCycle = true;
        }
    }

    // Temperature measurement
    float airTemp = AHT20.getTemperature();
    float airHumidity = AHT20.getHumidity();
    uint16_t tVOC = ENS160.getTVOC();
    uint16_t eCO2 = ENS160.getECO2();
    // Toggle heater

    // Carbon dioxide measurement
    // Toggle ventilation

    // Other VOCs measurements
    // Toggle warning LED


    // If temperature is higher, compute delta and decide heater time based on that.
    // Similar for pCO2.
    // Display current data to LCD after the measurement is done.
    if ( (millis() - myChronos.lcdSensors >= myDurations.lcdSensors)) {
        writeLcdSensors(soilMoistureMean, 7.1, 
                        airTemp, airHumidity,
                        eCO2, tVOC);
        myChronos.lcdSensors = millis();
        }

    if ( (millis() - myChronos.lcdLamp >= myDurations.lcdLamp)) {
        unsigned long lampTime = getLampTime(pinInputLight);
        writeLcdLamp(lampTime);
        myChronos.lcdLamp = millis();
    }

    
    // Create upload cycle loop
    // Simply replace by checking if data arrays are full? Upload when e.g. n=60 measurements have been taken +- 1 hour.
    // In Python webserver: get last timepoint of last update. Compute timedelta. Subdivide in n equal parts. Associate each measurement with its own timestamp.
    // Problem: not all measurements will have same delta. Soil every minute, air every 10 minutes.
    // DatetimeIndex + set_index + reindex + interpolate: https://stackoverflow.com/questions/38754132/pandas-interpolate-dataframe-with-new-length
    // Add lamp on/off bool to most frequent dataframe (soilMoisture)


}

void writeLcdStructure() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("GND    L/kg    pH");
    lcd.setCursor(0,1);
    lcd.print("AIR     \%       C");
    lcd.setCursor(0,2);
    lcd.print("PPM      CO2     VOC");
    lcd.setCursor(0,3);
    lcd.print("LMP     :       :");
}

void writeLcdSensors(float soilMoistureMean, float soilPh,
                     float airTemp, float airMoisture, 
                     uint16_t ppmCo2, uint16_t tVOC) {
    // Print the following message to the LCD:
    // GND 1.3Lkg 7.1pH 102     soilMoisture    soilPh      nextSoilMeas
    // AIR 99.8%  27.3C 102     airMoisture     airTemp     nextAirMeas
    // PPM 1023CO2  1011VOC     ppmCo2          ppmVoc
    // LMP   01h30  >08h30<     lmpTimeOn       lmpTimeOff

    // SOIL PARAMETERS
    lcd.setCursor(4, 0);
    if (myToggles.soilMeasurement == false) {
        lcd.print(soilMoistureMean, 1);
    } else {
        lcd.print("new");
    }
    lcd.setCursor(12, 0);
    lcd.print(soilPh, 1);
    lcd.setCursor(18, 0);
    lcd.print((myDurations.soilMeasurement - (millis() - myChronos.soilMeasurement))/1000);

    // AIR PARAMETERS
    lcd.setCursor(4, 1);
    lcd.print(airMoisture, 1);
    lcd.setCursor(11, 1);
    lcd.print(airTemp, 2);

    // GAS PARAMETERS
    lcd.setCursor(4, 2);
    if (ppmCo2 < 10000) {
        lcd.print(" ");
        if (ppmCo2 < 1000) {
            lcd.print(" ");
        }
    }
    lcd.print(ppmCo2);

    lcd.setCursor(13, 2);
    if (tVOC < 1000) {
        lcd.print(" ");
        if (tVOC < 100) {
            lcd.print(" ");
        }
    }
    lcd.print(tVOC);
}

void writeLcdLamp(unsigned long lampTime) {
    // LAMP PARAMETERS
    int quarters = lampTime / 15 / 60 / 1000;
    int remainder = 96 - quarters;
    lcd.setCursor(6, 3);
    lcd.print(formatTime(quarters));
    lcd.print("   ");
    lcd.print(formatTime(remainder));
}


String formatTime(int quarters) {
    String timeString = "";
    int displayHours = quarters / 4;
    int displayMins = 15 * (quarters % 4);
    if (displayHours < 10) {
        timeString += 0;
    }
    timeString += displayHours;
    timeString += ":";
    if (displayMins < 10) {
        timeString += 0;
    }
    timeString += displayMins;
    return timeString;
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