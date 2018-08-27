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
  uSonarRangeMsg.field_of_view = 0.1; //fake???
  uSonarRangeMsg.min_range = 5;
  uSonarRangeMsg.max_range = MAX_DISTANCE;
}

void loop() 
{
  if(millis() >= range_time)
  {
    uSonarRangeMsg.range = uSonar.ping_cm() / 100;
    uSonarRangeMsg.header.stamp = nh.now();
    pub_range.publish(&uSonarRangeMsg);
    range_time = millis() + 500;
  }

  nh.spinOnce();	// Is correct here? Or call only when have a new data?
}


