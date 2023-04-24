bool toggleForTime(int pin, unsigned long startTime, int duration, int defaultState) {
    // Toggle a pin ON/OFF for a given amount of time based on its default state (0: OFF, 1: ON).
    if (defaultState == 0) {
        digitalWrite(pin, HIGH);
    } else {
        digitalWrite(pin, LOW);
    }
    if millis() -  startTime >= duration {
        if (defaultState == 0) {
                digitalWrite(pin, LOW);
            } else {
                digitalWrite(pin, HIGH);
            }
        // The actuator is no longer active.
        return false;
    }
    // The actuator is still active.
    return true;
    }


float mapSensor(int pin, int voltage) {
    // Map the 10-bit pseudo-analog pin to the expected output voltage of the sensor.
    int sensorValue = analogRead(pin);
    float outputVoltage = (sensorValue / 1024.0) * float(voltage);
    return outputVoltage;
}


float linRegSensor(float reading, float slope, float intercept) {
    // Map an analyte concentration x to the sensor's voltage V using the following equation:
    // V(x) = Sx + I
    // x(V) = (V - I)/S
    return (reading - intercept) / slope;
}