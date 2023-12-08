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

//LogicalCameraImage temp information
osrf_gear::LogicalCameraImage camInfo [10];

void orderCallback(const osrf_gear::Order::ConstPtr& msg)
{
  // Add information to the end of the vector
  order_vector.push_back(*msg);
  ROS_INFO("%s order received", msg.order_id.c_str());
}


void cam_agv1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  camInfo [0] = *cam_msg;
}
void cam_agv2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  camInfo [1] = *cam_msg;
}
void cam_bin1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  camInfo [2] = *cam_msg;
}
void cam_bin2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  camInfo [3] = *cam_msg;
}
void cam_bin3_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  camInfo [4] = *cam_msg;
}
void cam_bin4_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  camInfo [5] = *cam_msg;
}
void cam_bin5_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  camInfo [6] = *cam_msg;
}
void cam_bin6_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  camInfo [7] = *cam_msg;
}
void cam_fau1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  camInfo [8] = *cam_msg;
}
void cam_fau2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & cam_msg) {
  camInfo [9] = *cam_msg;
}

// Retrieve the transformation
void arm_trans(std::string frame, geometry_msgs::TransformStamped tfStamped)
{ 
  try {
  tfStamped = tfBuffer.lookupTransform("arm1_base_link",frame,ros::Time(0.0), ros::Duration(1.0));
  ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
tfStamped.child_frame_id.c_str());
  } 
  catch (tf2::TransformException &ex) {
  ROS_ERROR("%s", ex.what());
  }
  // tf2_ross::Buffer.lookupTransform("to_frame", "from_frame", "how_recent","how_long_to_wait_for_transform");
}

geometry_msgs::PoseStamped Set_goal_pose()
{
    geometry_msgs::PoseStamped goal_pose;
    // Add height to the goal pose.
    goal_pose.pose.position.z += 0.10; // 10 cm above the part
    // Tell the end effector to rotate 90 degrees around the y-axis (in   quaternions...).
    goal_pose.pose.orientation.w = 0.707;
    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.707;
    goal_pose.pose.orientation.z = 0.0;

    return goal_pose;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_node");

  ros::NodeHandle n;


  std_srvs::Trigger begin_comp;
  // Declare the variable in this way where necessary in the code.
  std_srvs::SetBool my_bool_var;
  my_bool_var.request.data = true;
  // Create the service client.
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("service_node");
  // Variable to capture service call success.
  int service_call_succeeded;
  // Call the Service
  service_call_succeeded = begin_client.call(begin_comp);
  if(!begin_client.exists()) {
    ROS_INFO("Competition preparing ...");
  }
  ROS_INFO("Competition starting ...");
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


  // Clearing/initializing vector
  order_vector.clear();

  // Declare the transformation buffer to maintain a list of transformations
  tf2_ros::Buffer tfBuffer;
  // Instantiate a listener that listens to the tf and tf_static topics and to update the
  buffer.tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Subscriber oder_sub = n.subscribe<osrf_gear::Order>("/ariac/orders", 1000, orderserviceCallback); 

  ros::Subscriber cam_agv1_sub = n.subscribe("/ariac/logical_camera_agv1", 1000, cam_agv1_callback);
  ros::Subscriber cam_agv2_sub = n.subscribe("/ariac/logical_camera_agv2", 1000, cam_agv2_callback);

  ros::Subscriber cam_bin1_sub = n.subscribe("/ariac/logical_camera_bin1", 1000, cam_bin1_callback);
  ros::Subscriber cam_bin2_sub = n.subscribe("/ariac/logical_camera_bin2", 1000, cam_bin2_callback);
  ros::Subscriber cam_bin3_sub = n.subscribe("/ariac/logical_camera_bin3", 1000, cam_bin3_callback);
  ros::Subscriber cam_bin4_sub = n.subscribe("/ariac/logical_camera_bin4", 1000, cam_bin4_callback);
  ros::Subscriber cam_bin5_sub = n.subscribe("/ariac/logical_camera_bin5", 1000, cam_bin5_callback);
  ros::Subscriber cam_bin6_sub = n.subscribe("/ariac/logical_camera_bin6", 1000, cam_bin6_callback);
  
  ros::Subscriber cam_fau1_sub = n.subscribe("/ariac/quality_control_sensor_1", 1000, cam_faulty1_callback);
  ros::Subscriber cam_fau2_sub = n.subscribe("/ariac/quality_control_sensor_2", 1000, cam_faulty2_callback);

  ros::ServiceClient materialLocations =   n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

  int productnum = 0;
  int shipmentnum = 0;
  //order process
  if (orders.size() > 0)
{
    
    //process one product
    std::string type = orders_vector[0].shipments[shipmentnum].products[productnum].type;

    osrf_gear::GetMaterialLocations LocationsInfo;
    LocationsInfo.request.material_type = type;
    if (materialLocations.call(LocationsInfo)) 
    {
        std::string bins = LocationsInfo.response.storage_units[0].unit_id;
        if (bins == "bin1")
          {
             int bin = 2;
             std::string frame = "logical_camera_bin1_frame";
          }
        else if (bins == "bin2")
          {
             int bin = 3;
             std::string frame = "logical_camera_bin2_frame";
          }
        else if (bins == "bin3")
          {
             int bin = 4;
             std::string frame = "logical_camera_bin3_frame";
          }
        else if (bins == "bin4")
          {
             int bin = 5;
             std::string frame = "logical_camera_bin4_frame";
          }
        else if (bins == "bin5")
          {
             int bin = 6;
             std::string frame = "logical_camera_bin5_frame";
          }
        else if (bins == "bin6")
          {
             int bin = 7;
             std::string frame = "logical_camera_bin6_frame";
          }
        osrf_gear::LogicalCameraImage camInfo = camInfo[bin]

        geometry_msgs::TransformStamped tfStamped;
        arm_trans(frame, tfStamped)

        // Create variables
        geometry_msgs::PoseStamped part_pose, goal_pose;
        // Copy pose from the logical camera.
        part_pose.pose = camInfo.models[0].pose;
        goal_pose = Set_goal_pose();
       
        tf2::doTransform(part_pose, goal_pose, tfStamped);
    }

    ROS_INFO_STREAM("Product type is" + type);
    ROS_INFO_STREAM("Product unit is" + bins);
    ROS_INFO_STREAM("Product unit is" + part_pose.pose);


    // Another product
    if (productnum + 1 < sizeof(orders[0].shipments[productnum].products))
    {
        productnum ++;
    }
    else
    {
        productnum = 0;
        // Another shipment
        if (shipmentnum + 1 < sizeof(orders[0].shipments))
        {
            shipmentnum++;
        }
        else
        {
            ROS_INFO("End of order");
            shipmentnum = 0;
            orders.erase(orders.begin());
        }
    }
}


  return 0;
}
