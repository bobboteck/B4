/******************************************************************************
 * B4-Arduino
 * 
 * Created 17/09/2018 - By Roberto D'Amico
 * 
 * Description: this program measures the distance with the 3 US sensors and 
 * publishes the values as ROS messages
 * 
 * How to use: For more details on how to operate refer to the repository Wiki
 * 
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

long range_time;

NewPing uSonarLeft = NewPing(2,3,MAX_DISTANCE);
NewPing uSonarCenter = NewPing(4,5,MAX_DISTANCE);
NewPing uSonarRight = NewPing(6,7,MAX_DISTANCE);


// ROS - Node handle
ros::NodeHandle nh;
// ROS - Message type definition
sensor_msgs::Range uSonarLeftRangeMsg;
// ROS - Topics defintion
ros::Publisher pub_rangeLeft("topicSonarRangeLeft", &uSonarLeftRangeMsg);
ros::Publisher pub_rangeCenter("topicSonarRangeCenter", &uSonarCenterRangeMsg);
ros::Publisher pub_rangeRight("topicSonarRangeRight", &uSonarRightRangeMsg);

void setup() 
{
  //ROS
  nh.initNode();
  nh.advertise(pub_rangeLeft);
  nh.advertise(pub_rangeCenter);
  nh.advertise(pub_rangeRight);

  uSonarLeftRangeMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  uSonarLeftRangeMsg.header.frame_id = "frameSonarRangeLeft";
  uSonarLeftRangeMsg.field_of_view = 0.523599f;		//30 degrees in radians
  uSonarLeftRangeMsg.min_range = 0.05;    			// Meters
  uSonarLeftRangeMsg.max_range = 2;       			// Meters
  
  uSonarCenterRangeMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  uSonarCenterRangeMsg.header.frame_id = "frameSonarRangeCenter";
  uSonarCenterRangeMsg.field_of_view = 0.523599f;	//30 degrees in radians
  uSonarCenterRangeMsg.min_range = 0.05;    		// Meters
  uSonarCenterRangeMsg.max_range = 2;       		// Meters
  
  uSonarRightRangeMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  uSonarRightRangeMsg.header.frame_id = "frameSonarRangeRight";
  uSonarRightRangeMsg.field_of_view = 0.523599f;	//30 degrees in radians
  uSonarRightRangeMsg.min_range = 0.05;    			// Meters
  uSonarRightRangeMsg.max_range = 2;       			// Meters
}

void loop() 
{
  if(millis() >= range_time)
  {
    uSonarLeftRangeMsg.range = (float)uSonarLeft.ping_cm() / 100;
    uSonarLeftRangeMsg.header.stamp = nh.now();
    pub_rangeLeft.publish(&uSonarLeftRangeMsg);
	
	uSonarCenterRangeMsg.range = (float)uSonarCenter.ping_cm() / 100;
    uSonarCenterRangeMsg.header.stamp = nh.now();
    pub_rangeCenter.publish(&uSonarCenterRangeMsg);
	
	uSonarRightRangeMsg.range = (float)uSonarRight.ping_cm() / 100;
    uSonarRightRangeMsg.header.stamp = nh.now();
    pub_rangeRight.publish(&uSonarRightRangeMsg);
	
    range_time = millis() + 500;
  }

  nh.spinOnce();
}
