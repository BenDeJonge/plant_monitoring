
// -----------------------------------------------------------------------------
// I M P O R T S
// -----------------------------------------------------------------------------

#include <DFRobot_ENS160.h>
#include <AHT20.h>
#include <Sensor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
DFRobot_ENS160_I2C ENS160(&Wire, /*I2CAddr*/ 0x53);
AHT20 AHT20;

// -----------------------------------------------------------------------------
// G L O B A L S
// -----------------------------------------------------------------------------

// Heater of ENS160 and thermistor of AHT21 on same PCB
// Causes major fluctuations in temperature readings
// Cannot measure continuously. Put in idle mode inbetween readings.

// Timings
const int baudrate = 19200;


struct pinsInput {
    int soilMeasurement;
    int lamp;
} myInputs;

struct pinsOutput {
    int soilMeasurement;
    int watering;
    int airMeasurement;
    int lamp;
} myOutputs;

struct Chronos {
    // Soil measurement and watering event
    unsigned long soilMeasurement;
    unsigned long cycles;
    unsigned long watering;
    // Updating LCD
    unsigned long lcdSensors;
    unsigned long lcdLamp;
    // Lamp
    unsigned long lamp;
} myChronos;

struct Durations {
    // Soil measurement and watering event
    unsigned long soilMeasurement;
    unsigned long soilMeasurementLength;
    unsigned long cycles;
    unsigned long watering;
    // Updating LCD
    unsigned long lcdSensors;
    unsigned long lcdLamp;
} myDurations;

struct Toggles{
    bool firstRun;
    // Soil measurement and watering event
    bool soilMeasurement;
    bool watering;
    bool wateredThisCycle;
    // Lamp status
    bool lamp;
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

    // Initializations
    Serial.begin(baudrate);
    lcd.init();
    lcd.backlight();
    writeLcdStructure();
    ENS160.setPWRMode(ENS160_SLEEP_MODE);
    ENS160.setTempAndHum(/*temperature=*/25.0, /*humidity=*/50.0);
    
    // inputPins
    myInputs.soilMeasurement = A0;
    myInputs.lamp = A1;

    // outputPins
    myOutputs.soilMeasurement = 2;
    myOutputs.watering = 3;
    myOutputs.airMeasurement = 4;
    myOutputs.lamp = 5;

    // Chronos
    myChronos.soilMeasurement = 0;
    myChronos.cycles = 0;
    myChronos.watering = 0;
    myChronos.lcdSensors = 0;
    myChronos.lamp = 0;
    myChronos.lcdLamp = 0;

    // Durations
    myDurations.soilMeasurement = 60000;
    myDurations.soilMeasurementLength = 10000;
    myDurations.cycles = 25;
    myDurations.watering = 15000;
    myDurations.lcdSensors = 1000;
    myDurations.lcdLamp = 300;

    // Toggles
    myToggles.firstRun = true;
    myToggles.soilMeasurement = true;
    myToggles.watering = false;
    myToggles.wateredThisCycle = false;
    myToggles.lamp = true;

    // Calibrations
    myCalibs.slopeSoilMoisture = 1.0;
    myCalibs.interceptSoilMoisture = 0.0;
    myCalibs.thresholdSoilMoisture = 2.0;

    // Sensor readouts
    soilMoisture = 0.0;
    soilMoistureMean = 0.0;
    // Setting output pins and instantiating as low
    int pinsOutput [] = {myOutputs.soilMeasurement, myOutputs.watering, myOutputs.airMeasurement, myOutputs.lamp};
    for (int pin: pinsOutput) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    // Toggle air sensor on.
    digitalWrite(myOutputs.airMeasurement, HIGH);

    // Setting input pins
    int pinsInput [] = {myInputs.soilMeasurement, myInputs.lamp};
    for (int pin: pinsInput) {
        pinMode(pin, INPUT);
    }


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
        myChronos.cycles = 0;
        soilMoisture = 0.0;
        // Resetting the water states.
        myToggles.watering = false;
        myToggles.wateredThisCycle = false;
    }

    // Soil moisture measurement
    if (myToggles.soilMeasurement == true) {
        myToggles.soilMeasurement = toggleForTime(myOutputs.soilMeasurement,
                                                  myChronos.soilMeasurement, 
                                                  myDurations.soilMeasurementLength,
                                                  0);
        myChronos.cycles++;
        // Neglecting first measurements to charge capacitor
        if (myChronos.cycles > myDurations.cycles) {
            soilMoisture += linRegSensor(mapSensor(myInputs.soilMeasurement, 3),
                                        myCalibs.slopeSoilMoisture,
                                        myCalibs.interceptSoilMoisture);
        }
    // Measurement done. Compute mean
    } else {
        soilMoistureMean = soilMoisture / (myChronos.cycles - myDurations.cycles);
        // Toggle watering on if it is not yet on
        if ((soilMoistureMean >= myCalibs.thresholdSoilMoisture) && 
            (myToggles.watering == false)) {
            myToggles.watering = true;
            myChronos.watering = millis();
        }
    }

    // Trigger a watering event of fixed duration
    if ((myToggles.soilMeasurement == false) &&
        (myToggles.watering == true) &&
        (myToggles.wateredThisCycle == false)) {
            // Serial.print("Watering calls toggleForTime ");
            myToggles.watering = toggleForTime(myOutputs.watering, myChronos.watering, myDurations.watering, 0);
            // Reset the trigger if the watering event is over
            if (myToggles.watering == false) {
                myToggles.wateredThisCycle = true;
            }
    }

    // -------------------------------------------------------------------------
    // T O G G L I N G   H E A T E R
    // -------------------------------------------------------------------------

    // Temperature measurement
    float airTemp = AHT20.getTemperature();
    // Toggle heater

    // -------------------------------------------------------------------------
    // T O G G L I N G   V E N T I L A T I O N
    // -------------------------------------------------------------------------
    // Humidity and carbon dioxide measurement
    float airHumidity = AHT20.getHumidity();
    uint16_t tVOC = ENS160.getTVOC();
    // Toggle ventilation

    // Other VOCs measurements
    uint16_t eCO2 = ENS160.getECO2();
    // Toggle warning LED


    // If temperature is higher, compute delta and decide heater time based on that.
    // Similar for pCO2.

    // -------------------------------------------------------------------------
    // T O G G L I N G   L A M P
    // -------------------------------------------------------------------------

    // Amount of millis the lamp should be on.
    unsigned long lampTimeOn = getLampTime(myInputs.lamp);
    // Lamp is off
    if (myToggles.lamp == false) {
        // Lamp needs to turn on
        if ( millis() - myChronos.lamp >  86400000 - lampTimeOn) {
            myToggles.lamp = true;
            myChronos.lamp = millis();
        }
    // Lamp is on
    } else {
        // Check if it can toggle of yet
        myToggles.lamp = toggleForTime(myOutputs.lamp, myChronos.lamp, lampTimeOn, 0);
    }


    // -------------------------------------------------------------------------
    // D A T A   D I S P L A Y
    // -------------------------------------------------------------------------

    // Display current data to LCD after the measurement is done.
    if ( (millis() - myChronos.lcdSensors >= myDurations.lcdSensors) ) {
        writeLcdSensors(soilMoistureMean, 7.1, 
                        airTemp, airHumidity,
                        eCO2, tVOC);
        myChronos.lcdSensors = millis(); 
    }
    // Display current lamp time to LCD.
    if ( (millis() - myChronos.lcdLamp >= myDurations.lcdLamp) ) {
        writeLcdLamp(lampTimeOn, myToggles.lamp);
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
    // As long instead of unsigned long to avoid rollover issues when the passed time is slightly larger than the wanted time
    long remainder = ( (long) myDurations.soilMeasurement - (long) (millis() - myChronos.soilMeasurement) )/1000;
    if (remainder < 10) {
        lcd.print(" ");
    }
    lcd.print(remainder);

    // AIR PARAMETERS
    lcd.setCursor(4, 1);
    lcd.print(airMoisture, 1);
    lcd.setCursor(10, 1);
    if (airTemp > 0) {
        lcd.print(" ");
    }
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

void writeLcdLamp(unsigned long lampTime, bool lampOn) {
    // LAMP PARAMETERS
    int quarters = lampTime / 15 / 60 / 1000;
    int remainder = 96 - quarters;
    lcd.setCursor(5, 3);
    lcd.print(formatTime(quarters, lampOn));
    lcd.setCursor(13, 3);
    lcd.print(formatTime(remainder, !lampOn));
}


String formatTime(int quarters, bool activeState) {
    String timeString = "";
    if (activeState) {
        timeString += ">";
    } else {
        timeString += " ";
    }
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
    if (activeState) {
        timeString += "<";
    } else {
        timeString += " ";
    }
    return timeString;
} 

unsigned long getLampTime(int pinPotentiometer) {
    // Map the potentiometer voltage to an integer number of quarters. Return as millis.
    float outputVoltage = mapSensor(pinPotentiometer, 5);
    float quarters = outputVoltage * (96.0 / 5.0);
    unsigned long lampTime = round(quarters) * 15 * 60 * 1000;
    return lampTime;
}