#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include <sstream>
// Transformation header files
#include "tf2_ros/tranform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

// Declaring a vector of data type.
std::vector<osrf_gear::Order> order_vector;

void orderCallback(const std::vector::Order& msg)
{
  ROS_INFO_STREAM("order_id is ", msg.order_id);
  ROS_INFO_STREAM("Product type is ", msg.Product[].type);
  ROS_INFO_STREAM("Product type is ", msg.pose);
  // Add information to the end of the vector
  order_vector.push_back(msg);
}

void transformation_function()
{
  // Declare the transformation buffer to maintain a list of transformations
  tf2_ros::Buffer tfBuffer;
  // Instantiate a listener that listens to the tf and tf_static topics and to update the
  buffer.tf2_ros::TransformListener tfListener(tfBuffer);
  // Retrieve the transformation
  geometry_msgs::TransformStamped tfStamped;
  try {
  tfStamped = tfBuffer.lookupTransform("arm1_base_link",   "logical_camera_bin4_frame",ros::Time(0.0), ros::Duration(1.0));
  ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
tfStamped.child_frame_id.c_str());
} 
  catch (tf2::TransformException &ex) {
  ROS_ERROR("%s", ex.what());
}
// tf2_ross::Buffer.lookupTransform("to_frame", "from_frame", "how_recent","how_long_to_wait_for_transform");
 
  // Create variables
  geometry_msgs::PoseStamped part_pose, goal_pose;
  // Copy pose from the logical camera.
  part_pose.pose = ...;
  tf2::doTransform(part_pose, goal_pose, transformStamped);

  // Add height to the goal pose.
  goal_pose.pose.position.z += 0.10; // 10 cm above the part
  // Tell the end effector to rotate 90 degrees around the y-axis (in   quaternions...).
  goal_pose.pose.orientation.w = 0.707;
  goal_pose.pose.orientation.x = 0.0;
  goal_pose.pose.orientation.y = 0.707;
  goal_pose.pose.orientation.z = 0.0;
 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_node");

  ros::NodeHandle n;
  std_srvs::Trigger begin_comp;

  // Declare the variable in this way where necessary in the code.
  std_srvs::SetBool my_bool_var;
  my_bool_var.request.data = true;

  // Clearing/initializing vector
  order_vector.clear();

  ros::Subscriber oder_sub = n.subscribe<osrf_gear::Order>("/ariac/orders", 1000, orderserviceCallback);

  // Create the service client.
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("service_node");

ros::Rate loop_rate(10);

  while (ros::ok())
  {

    // Variable to capture service call success.
    int service_call_succeeded;
    // Call the Service
    service_call_succeeded = begin_client.call(begin_comp);
    if(!service_call_succeeded)
    {
      ROS_ERROR("Competition service call failed! Goodness Gracious!!");
      ROS_WARN("Competition service returned failure: %s",   begin_comp.response.message.c_str());
    }
    else 
    {
       ROS_INFO("Competition service called successfully: %s", \
begin_comp.response.message.c_str());
    }
  }


  return 0;
}
