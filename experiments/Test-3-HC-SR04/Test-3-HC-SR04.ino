/****************************************************************************** 
 * This example use the simple Example of libraries NewPing and U8g2lib to show
 * data of 3 US sensor HC-SR04 on a 0.96" Oled display (I2C) and on Arduino
 * IDE Serial Plot or Serial Monitor. You can use only a Display or Serial 
 * connection to test this code.
 * 
 * N.B. All code are not optimized, is simply for test functionality!
 *****************************************************************************/
#include <U8g2lib.h>
#include <NewPing.h>
#include <stdlib.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(2, 3, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(4, 5, MAX_DISTANCE), 
  NewPing(6, 7, MAX_DISTANCE)
};

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void setup() 
{
  // Open serial monitor at 115200 baud to see ping results.
  Serial.begin(115200);
  // Inizialize Diplay
  u8g2.begin();
}

void loop() 
{
  u8g2.clearBuffer();
  u8g2_FontSettings();
  // Show first row with Title
  u8g2.drawStr(0,0, "HC-SR04");
  
  for (uint8_t i = 0; i < SONAR_NUM; i++) // Loop through each sensor and display results.
  { 
    // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    delay(50);
    uint8_t sonar_temp = sonar[i].ping_cm();
    
    // Use this to show data in to Arduino IDE Serial Monitor
    Serial.print(i);
    Serial.print("=");
    Serial.print(sonar_temp);
    Serial.print("cm ");
    // Use this to show data in to Arduino IDE Serial Plot
    Serial.print(sonar_temp);
    Serial.print(",");

    // Convert sensor mesure in string to punt it in display
    char index[3];
    itoa((int)i, index, 10);

    char usRange[3];
    itoa((int)sonar_temp, usRange, 10);

    // Write string as index:range 
    u8g2.setFont(u8g2_font_9x15_mf);
    u8g2.drawStr( 0, 16+(16*i), index);
    u8g2.drawStr(20, 16+(16*i), ":");
    u8g2.drawStr(40, 16+(16*i), usRange);
  }
  //u8g2.sendBuffer();
  Serial.println();
}

void u8g2_FontSettings(void) 
{
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}
