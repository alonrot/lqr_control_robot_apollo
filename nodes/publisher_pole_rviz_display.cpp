#include "ros/ros.h" 
#include "ros/master.h"
#include "tf/LinearMath/Quaternion.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"


#define SUBSCRIBE_TOPIC "/pole_angle/visualization"
#define BALL_PUBLISH_TOPIC "pole_balancing/pole_display/ball"
#define BALL_BASE_PUBLISH_TOPIC "pole_balancing/pole_display/ball_base"
#define STICK_PUBLISH_TOPIC "pole_balancing/pole_display/stick"
#define HANDLE_PUBLISH_TOPIC "pole_balancing/pole_display/handle"
#define FRAME_ID "R_PALM"
#define PI 3.14159
#define ANGLE_OFFSET 0
// stick
#define POLE_STICK_LENGTH 1 // in meters
#define POLE_STICK_RED 173/255.0
#define POLE_STICK_GREEN 216/255.0
#define POLE_STICK_BLUE 230/255.0
#define POLE_STICK_DIAMETER 0.01
// handle
#define POLE_HANDLE_LENGTH 0.18 // in meters
#define POLE_HANDLE_RED 70/255.0
#define POLE_HANDLE_GREEN 130/255.0
#define POLE_HANDLE_BLUE 180/255.0
#define POLE_HANDLE_DIAMETER 0.01
// ball
#define POLE_BALL_RED 70/255.0
#define POLE_BALL_GREEN 130/255.0
#define POLE_BALL_BLUE 180/255.0
#define POLE_BALL_DIAMETER 0.1


// documentation on markers: wiki.ros.org/rviz/DisplayTypes/Marker
// note: I tried to use markerArray with a single publisher, but 
// could not get rviz to display it. no idea why.


static ros::NodeHandle *node;
static ros::Publisher ball_publisher;
static ros::Publisher ball_base_publisher;
static ros::Publisher handle_publisher;
static ros::Publisher stick_publisher;

static visualization_msgs::Marker pole_handle;
static visualization_msgs::Marker pole_stick;
static visualization_msgs::Marker pole_ball;
static visualization_msgs::Marker pole_ball_base;


// d is the position on the pole, e.g. 0 for handle, POLE_STICK_LENGTH for pole tip
static geometry_msgs::Point _get_coord_in_endeff_frame(double d, double angle, double pole_handle_length, double pole_handle_diameter){

  geometry_msgs::Point point;
  point.x = -d*cos(angle)-pole_handle_length/2.0;
  point.y = -d*sin(angle);
  point.z = pole_handle_diameter/2.0;
  return point;

}

void pole_angle_callback(std_msgs::Float32 angle_msg){

  float angle = angle_msg.data + ANGLE_OFFSET;

  geometry_msgs::Point pole_base;  
  pole_base.x = -POLE_HANDLE_LENGTH/2.0;
  pole_base.y = 0.0;
  pole_base.z = POLE_HANDLE_DIAMETER/2.0;

  geometry_msgs::Point pole_tip = _get_coord_in_endeff_frame(POLE_STICK_LENGTH, angle, POLE_HANDLE_LENGTH,POLE_HANDLE_DIAMETER);

  // Get the time stamp only once:
  ros::Time t_cur = ros::Time::now();

  // ball
  pole_ball.pose.position = pole_tip;
  pole_ball.header.stamp = t_cur;
  ball_publisher.publish(pole_ball);

  // stick
  pole_stick.points.clear();
  pole_stick.points.push_back(pole_base);
  pole_stick.points.push_back(pole_tip);
  pole_stick.header.stamp = t_cur;

  // handle
  pole_handle.header.stamp = t_cur;
  
  // ball at the base
  pole_ball_base.header.stamp = t_cur;

  // publishing
  ball_publisher.publish(pole_ball);
  ball_base_publisher.publish(pole_ball_base);
  handle_publisher.publish(pole_handle);
  stick_publisher.publish(pole_stick);

}


int main(int argc, char **argv){

  // initializing broadcasted messages
  // pole stick
  pole_stick.type = visualization_msgs::Marker::ARROW;
  pole_stick.action = visualization_msgs::Marker::ADD;
  pole_stick.scale.x = POLE_STICK_DIAMETER;
  pole_stick.scale.y = POLE_STICK_DIAMETER;
  pole_stick.scale.z = 0.001;
  pole_stick.color.r = POLE_STICK_RED;
  pole_stick.color.g = POLE_STICK_GREEN;
  pole_stick.color.b = POLE_STICK_BLUE;
  pole_stick.color.a = 1;
  pole_stick.header.frame_id = FRAME_ID;
  // pole handle
  pole_handle.type = visualization_msgs::Marker::ARROW;
  pole_handle.action = visualization_msgs::Marker::ADD;
  pole_handle.scale.x = POLE_HANDLE_DIAMETER;
  pole_handle.scale.y = 0;
  pole_handle.scale.z = 0.001;
  pole_handle.color.r = POLE_HANDLE_RED;
  pole_handle.color.g = POLE_HANDLE_GREEN;
  pole_handle.color.b = POLE_HANDLE_BLUE;
  pole_handle.color.a = 1;
  // position and orientation of the handle is fixed at origin (in end effector frame)
  geometry_msgs::Point handle_base;
  handle_base.z = POLE_HANDLE_DIAMETER/2.0 ; handle_base.y = 0.0;
  handle_base.x = -POLE_HANDLE_LENGTH/2.0;
  geometry_msgs::Point handle_tip;
  handle_tip.z = POLE_HANDLE_DIAMETER/2.0 ; handle_tip.y = 0.0;
  handle_tip.x = +POLE_HANDLE_LENGTH/2.0;
  pole_handle.points.push_back(handle_base);
  pole_handle.points.push_back(handle_tip);
  pole_handle.header.frame_id = FRAME_ID;
  // pole ball on at base of the pole
  pole_ball_base.type = visualization_msgs::Marker::SPHERE;
  pole_ball_base.action = visualization_msgs::Marker::ADD;
  pole_ball_base.scale.x = POLE_HANDLE_DIAMETER*1.5;
  pole_ball_base.scale.y = POLE_HANDLE_DIAMETER*1.5;
  pole_ball_base.scale.z = POLE_HANDLE_DIAMETER*1.5;
  pole_ball_base.color.r = POLE_STICK_RED;
  pole_ball_base.color.g = POLE_STICK_GREEN;
  pole_ball_base.color.b = POLE_STICK_BLUE;
  pole_ball_base.color.a = 1;
  // ball at the base position is fixed
  pole_ball_base.pose.position.x = -POLE_HANDLE_LENGTH/2.0;
  pole_ball_base.pose.position.y = 0;
  pole_ball_base.pose.position.z = POLE_HANDLE_DIAMETER/2.0;
  pole_ball_base.header.frame_id = FRAME_ID;
  // pole ball on the tip of the pole
  pole_ball.type = visualization_msgs::Marker::SPHERE;
  pole_ball.action = visualization_msgs::Marker::ADD;
  pole_ball.scale.x = POLE_BALL_DIAMETER;
  pole_ball.scale.y = POLE_BALL_DIAMETER;
  pole_ball.scale.z = POLE_BALL_DIAMETER;
  pole_ball.color.r = POLE_BALL_RED;
  pole_ball.color.g = POLE_BALL_GREEN;
  pole_ball.color.b = POLE_BALL_BLUE;
  pole_ball.color.a = 1;
  pole_ball.header.frame_id = FRAME_ID;

  // initialize ros stuff
  ros::init(argc,argv,"pole_visualization_rviz");
  node = new ros::NodeHandle();
  ball_publisher      = node->advertise<visualization_msgs::Marker>(BALL_PUBLISH_TOPIC,5);
  ball_base_publisher = node->advertise<visualization_msgs::Marker>(BALL_BASE_PUBLISH_TOPIC,5);
  handle_publisher    = node->advertise<visualization_msgs::Marker>(HANDLE_PUBLISH_TOPIC,5);
  stick_publisher     = node->advertise<visualization_msgs::Marker>(STICK_PUBLISH_TOPIC,5);
  ros::Subscriber pole_angle = node->subscribe(SUBSCRIBE_TOPIC,5,pole_angle_callback);

  // running
  ros::spin();


  return 0;

}

