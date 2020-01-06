/*
 * Name          : HC-SR04-LPFilter.ino
 * @author       : Roberto D'Amico (Bobboteck)
 * Last modified : 27.12.2019
 * Revision      : 0.0.1
 *
 * Modification History:
 * Date         Version     Modified By        Description
 * 2019-12-27   0.0.1       Roberto D'Amico    First version of code
 * 
 * Description:
 * This Experiment use 3 US sensor HC-SR04, for each sensor signal was applied
 * an Low-Pass-Filter to increase the stability of signal.
 * If you open the "Serial Plotter" of the "Arduino IDE", you can see for each
 * sensor the RAW value and the Filtered value.
 * 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Roberto D'Amico (Bobboteck)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <NewPing.h>
#include <stdlib.h>
#include <Lpf.h>

#define SONAR_NUM       3       // Number of sensors.
#define MAX_DISTANCE    200     // Maximum distance (in cm) to ping.

// Sensor object array. Each sensor's trigger pin, echo pin, and max distance to ping.
NewPing sonar[SONAR_NUM] = 
{
    NewPing(2, 3, MAX_DISTANCE),  // Left 
    NewPing(4, 5, MAX_DISTANCE),  // Center
    NewPing(6, 7, MAX_DISTANCE)   // Right
};

LPF lpf[SONAR_NUM] =
{
    LPF(0.2,IS_BANDWIDTH_HZ),
    LPF(0.2,IS_BANDWIDTH_HZ),
    LPF(0.2,IS_BANDWIDTH_HZ)
};

void setup() 
{
    // Open serial monitor at 115200 baud to see ping results.
    Serial.begin(115200);
}

void loop() 
{
    // Loop through each sensor.
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    { 
        delay(200);
        uint8_t sonar_temp = sonar[i].ping_cm();
    
        double lpfValue = lpf[i].NextValue(sonar_temp);

        // Use this to show data in to Arduino IDE Serial Plot
        Serial.print(sonar_temp);
        Serial.print(",");
        Serial.print(lpfValue);
        Serial.print(",");
  }

  Serial.println();
}
