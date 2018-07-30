//#include <U8g2lib.h>
#include <NewPing.h>
//#include <stdlib.h>

//#ifdef U8X8_HAVE_HW_I2C
//#include <Wire.h>
//#endif

#include <ros.h>
#include <std_msgs/UInt8.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(2, 3, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(4, 5, MAX_DISTANCE), 
  NewPing(6, 7, MAX_DISTANCE)
};

//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//ROS
ros::NodeHandle nh;

std_msgs::UInt8 SonarOneRange;
ros::Publisher p("SonarOneRange_topic", &SonarOneRange);

void setup() 
{
  //Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  //u8g2.begin();

  //ROS
  nh.initNode();
  nh.advertise(p);
}

void loop() 
{
/*  u8g2.clearBuffer();
  u8g2_FontSettings();
  // Mostra a video il Titolo
  u8g2.drawStr(0,0, "HC-SR04");
  
  for (uint8_t i = 0; i < SONAR_NUM; i++) // Loop through each sensor and display results.
  { 
    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    //uint8_t sonar_temp = sonar[i].ping_cm();
    
    Serial.print(i);
    Serial.print("=");
    Serial.print(sonar_temp);
    Serial.print("cm ");
    Serial.print(sonar_temp);
    Serial.print(",");

    char index[3];
    itoa((int)i, index, 10);

    char misura[3];
    itoa((int)sonar[i].ping_cm(), misura, 10);

    // Stampa la riga con 
    u8g2.setFont(u8g2_font_9x15_mf);
    u8g2.drawStr( 0, 16+(16*i), index);
    u8g2.drawStr(20, 16+(16*i), ":");
    u8g2.drawStr(40, 16+(16*i), misura);
  }
  u8g2.sendBuffer();
  //Serial.println();
*/

  //ROS
  SonarOneRange.data=sonar[0].ping_cm();
  p.publish(&SonarOneRange);
  nh.spinOnce();

  delay(1000);
}
/*
void u8g2_FontSettings(void) 
{
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}*/

