/****************************************************************************** 
 * This example use the simple Example of libraries NewPing and U8g2lib to show
 * data of 3 US sensor HC-SR04 on a 0.96" Oled display (I2C) and on Arduino
 * IDE Serial Plot or Serial Monitor. You can use only a Display or Serial 
 * connection to test this code.
 * 
 * N.B. All code are not optimized, is simply for test functionality!
 *****************************************************************************/
#include <NewPing.h>
#include <stdlib.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

// Sensor object array. Each sensor's trigger pin, echo pin, and max distance to ping.
NewPing sonar[SONAR_NUM] = 
{
  NewPing(2, 3, MAX_DISTANCE),  // Left 
  NewPing(4, 5, MAX_DISTANCE),  // Center
  NewPing(6, 7, MAX_DISTANCE)   // Right
};

void setup() 
{
    // Open serial monitor at 115200 baud to see ping results.
    Serial.begin(115200);
}

void loop() 
{
  for (uint8_t i = 0; i < SONAR_NUM; i++) // Loop through each sensor and display results.
  { 
    delay(100);
    uint8_t sonar_temp = sonar[i].ping_cm();
    
    // Use this to show data in to Arduino IDE Serial Plot
    Serial.print(sonar_temp);
    Serial.print(",");
  }

  Serial.println();
}
