#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

int main(int argc, char  **argv) {
  ros::NodeHandle n;
  ros::Publisher wrench_topic = n.advertise<geometry_msgs::WrenchStamped>("wrench", 1);
  ros::Rate loop_rate(10);

  int count = 0; // in case you wanna count the msgs you sent
  geometry_msgs::WrenchStamped rnd_wrench;

  while(ros::ok()){
    rnd_wrench.wrench.force.x = 0;
    rnd_wrench.wrench.force.y = 0;
    rnd_wrench.wrench.force.z = 1;
    rnd_wrench.wrench.torque.x = 0;
    rnd_wrench.wrench.torque.y = 0;
    rnd_wrench.wrench.torque.z = 0;

    wrench_topic.publish(rnd_wrench);
    ros::spinOnce();
  }
  return 0;
}
