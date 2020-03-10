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
 * 2020-03-09   0.1.1       Roberto D'Amico     Fixed same errors in ROS object name for Sonar
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

// Array of topics name
char *topicName[] = { "topicSonarRangeLeft", "topicSonarRangeCenter", "topicSonarRangeRight" };

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
sensor_msgs::Range rangeMessage[SONAR_NUM] =
{
    sensor_msgs::Range(),
    sensor_msgs::Range(),
    sensor_msgs::Range()
};

// ROS - Topics defintion
ros::Publisher publisherRange[SONAR_NUM] =
{
    ros::Publisher(topicName[0], &rangeMessage[0]),
    ros::Publisher(topicName[1], &rangeMessage[1]),
    ros::Publisher(topicName[2], &rangeMessage[2])
};

void setup() 
{
    // ROS - initialize node
    nh.initNode();
    // Message definition/configuration
    for(uint8_t i=0;i<SONAR_NUM;i++)
    {
        nh.advertise(publisherRange[i]);

        rangeMessage[i].radiation_type = sensor_msgs::Range::ULTRASOUND;
        rangeMessage[i].header.frame_id = topicName[i];
        rangeMessage[i].field_of_view = 0.523599f;		//30 degrees in radians
        rangeMessage[i].min_range = 0.05;    			// Meters
        rangeMessage[i].max_range = 2;       			// Meters
    }
}

void loop() 
{
    // Loop through each sensor
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    { 
        delay(200);
        // Read sensor value
        uint8_t sonar_temp = sonar[i].ping_cm();
        // Apply low pass filter
        double lpfValue = lpf[i].NextValue(sonar_temp);
        // Add info to message and send it
        rangeMessage[i].range = (float)lpfValue / 100;
        rangeMessage[i].header.stamp = nh.now();
        publisherRange[i].publish(&rangeMessage[i]);
    }

    nh.spinOnce();
}
