/*
 * Name          : b4-arduino.ino
 * @author       : Roberto D'Amico (Bobboteck)
 * Last modified : 01.01.2020
 * Revision      : 0.1.0
 *
 * Modification History:
 * Date         Version     Modified By         Description
 * 2018-09-17   0.0.1       Roberto D'Amico     First version of code
 * 2020-01-01   0.1.0       Roberto D'Amico     Added filter functionality
 * 
 * Description:
 * This program use 3 US sensor HC-SR04, for each sensor signal was applied
 * an Low-Pass-Filter to increase the stability of signal and publishes the 
 * values as ROS messages.
 * 
 * How to use:
 * For more details on how to operate refer to the repository Wiki on GitHub
 * https://github.com/bobboteck/B4
 * 
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Roberto D'Amico (Bobboteck)
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
#include <Lpf.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

#define SONAR_NUM       3       // Number of sensors
#define MAX_DISTANCE    200     // Maximum distance (in cm) to ping

//long range_time;

// Sensor object array. For each sensor was specified trigger pin, echo pin, and max distance to ping
NewPing sonar[SONAR_NUM] = 
{
    NewPing(2, 3, MAX_DISTANCE),  // Left 
    NewPing(4, 5, MAX_DISTANCE),  // Center
    NewPing(6, 7, MAX_DISTANCE)   // Right
};

// Low Pass Filter array. For each sensor was initialized one costructor
LPF lpf[SONAR_NUM] =
{
    LPF(0.2,IS_BANDWIDTH_HZ),
    LPF(0.2,IS_BANDWIDTH_HZ),
    LPF(0.2,IS_BANDWIDTH_HZ)
};

// ROS - Node handle
ros::NodeHandle nh;

// ROS - Message type definition
//sensor_msgs::Range uSonarLeftRangeMsg;
sensor_msgs::Range rangeMessage[SONAR_NUM] =
{
    sensor_msgs::Range(),
    sensor_msgs::Range(),
    sensor_msgs::Range()
};

// ROS - Topics defintion
//ros::Publisher pub_rangeLeft("topicSonarRangeLeft", &uSonarLeftRangeMsg);
//ros::Publisher pub_rangeCenter("topicSonarRangeCenter", &uSonarCenterRangeMsg);
//ros::Publisher pub_rangeRight("topicSonarRangeRight", &uSonarRightRangeMsg);

ros::Publisher publisherRange[SONAR_NUM] =
{
    ros::Publisher("topicSonarRangeLeft", &rangeMessage[0]),
    ros::Publisher("topicSonarRangeCenter", &rangeMessage[1]),
    ros::Publisher("topicSonarRangeRight", &rangeMessage[2]),
};

void setup() 
{
    //ROS
    nh.initNode();
  //nh.advertise(pub_rangeLeft);
  //nh.advertise(pub_rangeCenter);
  //nh.advertise(pub_rangeRight);

    for(uint8_t i=0;i<SONAR_NUM;i++)
    {
        nh.advertise(publisherRange[i]);

        rangeMessage[i].radiation_type = sensor_msgs::Range::ULTRASOUND;
        rangeMessage[i].header.frame_id = "frameSonarRangeLeft";
        rangeMessage[i].field_of_view = 0.523599f;		//30 degrees in radians
        rangeMessage[i].min_range = 0.05;    			// Meters
        rangeMessage[i].max_range = 2;       			// Meters
    }

/*
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
  */
}

void loop() 
{
  /*if(millis() >= range_time)
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
  }*/


    // Loop through each sensor
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    { 
        delay(200);
        //
        uint8_t sonar_temp = sonar[i].ping_cm();
        //
        double lpfValue = lpf[i].NextValue(sonar_temp);
        //
        rangeMessage[i].range = (float)lpfValue / 100;
        rangeMessage[i].header.stamp = nh.now();
        publisherRange[i].publish(&rangeMessage[i]);
    }

    nh.spinOnce();
}
