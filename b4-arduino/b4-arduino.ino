/******************************************************************************
 * B4-Arduino
 * 
 * Created 01/09/2018 - By Roberto D'Amico
 * 
 * Description: this program measures the distance with the 3 US sensors and 
 * publishes the values as ROS messages
 * 
 * How to use: For more details on how to operate refer to the repository Wiki
 * at https://github.com/bobboteck/B4/wiki/ROS-Range-HC-SR04
 * 
 * 
 * MIT License
 * 
 * Copyright (c) 2018 LinFA
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *****************************************************************************/ 

#include <NewPing.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

char frameid[] = "uSonarRange_topic";
long range_time;

NewPing uSonar = NewPing(2,3,MAX_DISTANCE);


//ROS
ros::NodeHandle nh;

sensor_msgs::Range uSonarRangeMsg;
ros::Publisher pub_range("uSonarRange_topic", &uSonarRangeMsg);

void setup() 
{
  //ROS
  nh.initNode();
  nh.advertise(pub_range);

  uSonarRangeMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  uSonarRangeMsg.header.frame_id = frameid;
  uSonarRangeMsg.field_of_view = 0.523599f; //30 degrees in radians
  uSonarRangeMsg.min_range = 0.05;    // Meters
  uSonarRangeMsg.max_range = 2;       // Meters
}

void loop() 
{
  if(millis() >= range_time)
  {
    uSonarRangeMsg.range = (float)uSonar.ping_cm() / 100;
    uSonarRangeMsg.header.stamp = nh.now();
    pub_range.publish(&uSonarRangeMsg);
    range_time = millis() + 500;
  }

  nh.spinOnce();	// Is correct here? Or call only when have a new data?
}
