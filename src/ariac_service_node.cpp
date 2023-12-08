#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include <sstream>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "sensor_msgs/JointState.h"
#include "ik_service/PoseIK.h"
#include "trajectory_msgs/JointTrajectory.h"
// Action Server headers
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
// The Action Server "message type"
#include "control_msgs/FollowJointTrajectoryAction.h"


// Declaring a vector of data type.
std::vector<osrf_gear::Order> order_vector;

//LogicalCameraImage temp information
osrf_gear::LogicalCameraImage camInfo [10];

//Current joint information for arm1
sensor_msgs::JointState joint_current;

void ordercallback(const osrf_gear::Order::ConstPtr& msg)
{
  // Add information to the end of the vector
  order_vector.push_back(*msg);
  ROS_INFO("%s order received",order_vector.back().order_id.c_str());
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

void joint_global_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_current = *msg;
}

// Retrieve the transformation
void arm_trans(std::string frame, geometry_msgs::TransformStamped tfStamped)
{ 
  // Declare the transformation buffer to maintain a list of transformations
  tf2_ros::Buffer tfBuffer;
  // Instantiate a listener that listens to the tf and tf_static topics and to update the
  tf2_ros::TransformListener tfListener(tfBuffer);

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

void move_arm_to(int count, sensor_msgs::JointState joint_current, geometry_msgs::PoseStamped Pose, ros::Publisher follow_joint_trajectory)
{
  // Declare a variable for generating and publishing a trajectory.
  trajectory_msgs::JointTrajectory joint_trajectory;
  // Fill out the joint trajectory header.
  // Each joint trajectory should have an non-monotonically increasing sequence number.
  joint_trajectory.header.seq = count++;
  joint_trajectory.header.stamp = ros::Time::now(); 
  joint_trajectory.header.frame_id = "/world"; // Frame in which this is specified.
  // Set the names of the joints being used. All must be present.
  joint_trajectory.joint_names.clear();
  joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
  joint_trajectory.joint_names.push_back("shoulder_pan_joint");
  joint_trajectory.joint_names.push_back("shoulder_lift_joint");
  joint_trajectory.joint_names.push_back("elbow_joint");
  joint_trajectory.joint_names.push_back("wrist_1_joint");
  joint_trajectory.joint_names.push_back("wrist_2_joint");
  joint_trajectory.joint_names.push_back("wrist_3_joint");
  // Set a start and end point.
  joint_trajectory.points.resize(2);
  // Set the start point to the current position of the joints from joint_states.
joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
  for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) 
  {
   for (int indz = 0; indz < joint_current.name.size(); indz++) 
   {
     if (joint_trajectory.joint_names[indy] == joint_current.name[indz]) 
       {
           joint_trajectory.points[0].positions[indy] = joint_current.position[indz];
            break;
       }
     }
  }
  // When to start (immediately upon receipt).
  joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
  // Must select which of the num_sols solutions to use. Just start with the first.
  int q_des_indx = 0;
  // Set the end point for the movement
joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
  // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
joint_trajectory.points[1].positions[0] = joint_current.position[1];
  // The actuators are commanded in an odd order, enter the joint positions in the correct positions
  for (int indy = 0; indy < 6; indy++) 
  {
    joint_trajectory.points[1].positions[indy + 1] = Pose.pose.position[indy];
  }
  // How long to take for the movement.
  joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
  follow_joint_trajectory.publish(joint_trajectory);
}


//void end_effector(double T_pose, double q_pose, sensor_msgs::JointState joint_current)
//{
  // Where is the end effector given the joint angles.
  // joint_states.position[0] is the linear_arm_actuator_joint
  //q_pose[0] = joint_current.position[1];
  //q_pose[1] = joint_current.position[2];
  //q_pose[2] = joint_current.position[3];
  //q_pose[3] = joint_current.position[4];
  //q_pose[4] = joint_current.position[5];
  //q_pose[5] = joint_current.position[6];
  //ur_kinematics::forward((float *)&q_pose, (double *)&T_pose);
//}

//void joint_angles(double T_des, double q_des, trajectory_msgs::JointTrajectory desired)
//{
  // What joint angles put the end effector at a specific place.
  // Desired pose of the end effector wrt the base_link.
  //T_des[0][3] = desired.pose.position.x;
  //T_des[1][3] = desired.pose.position.y;
  //T_des[2][3] = desired.pose.position.z + 0.3; // above part
  //T_des[3][3] = 1.0;
  // The orientation of the end effector so that the end effector is down.
  //T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
  //T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
  //T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
  //T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;
  //int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);
//}

// This callback is called when a goal becomes active.
//void goalActiveCallback() {

//}
// This callback is called when feedback is provided
//void feedbackCallback(const control_msgs::JointTrajectoryFeedbackConstPtr& fb) {

//}
// This callback is called when the action is complete and provides the result
//void resultCallback(const control_msgs::JointTrajectoryResultConstPtr& res) {

//}


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

  ros::Rate loop_rate(10);

  // Clearing/initializing vector
  order_vector.clear();

  ros::Subscriber order_sub = n.subscribe<osrf_gear::Order>("/ariac/orders", 1000, ordercallback); 

  ros::Subscriber cam_agv1_sub = n.subscribe("/ariac/logical_camera_agv1", 1000, cam_agv1_callback);
  ros::Subscriber cam_agv2_sub = n.subscribe("/ariac/logical_camera_agv2", 1000, cam_agv2_callback);

  ros::Subscriber cam_bin1_sub = n.subscribe("/ariac/logical_camera_bin1", 1000, cam_bin1_callback);
  ros::Subscriber cam_bin2_sub = n.subscribe("/ariac/logical_camera_bin2", 1000, cam_bin2_callback);
  ros::Subscriber cam_bin3_sub = n.subscribe("/ariac/logical_camera_bin3", 1000, cam_bin3_callback);
  ros::Subscriber cam_bin4_sub = n.subscribe("/ariac/logical_camera_bin4", 1000, cam_bin4_callback);
  ros::Subscriber cam_bin5_sub = n.subscribe("/ariac/logical_camera_bin5", 1000, cam_bin5_callback);
  ros::Subscriber cam_bin6_sub = n.subscribe("/ariac/logical_camera_bin6", 1000, cam_bin6_callback);
  
  ros::Subscriber cam_fau1_sub = n.subscribe("/ariac/quality_control_sensor_1", 1000, cam_fau1_callback);
  ros::Subscriber cam_fau2_sub = n.subscribe("/ariac/quality_control_sensor_2", 1000, cam_fau2_callback);

  ros::Subscriber arm1_joint = n.subscribe("/ariac/arm1/joint_states", 10, joint_global_callback);

  ros::ServiceClient materialLocations =   n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

  ros::ServiceClient ik_pose = n.serviceClient<ik_service::PoseIK>("/pose_ik_service");
  ros::service::waitForService("/pose_ik_service", 100);

  ros::Publisher follow_joint_trajectory = n.advertise<trajectory_msgs::JointTrajectory>("ariac/arm1/arm/command", 1000);
  
  // Instantiate the Action Server client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> \
trajectory_ac("ariac/arm/follow_joint_trajectory", true);

  ros::AsyncSpinner spinner(1); // Use 1 thread
  spinner.start(); // A spinner makes calling ros::spin() unnecessary.

  int productnum = 0;
  int shipmentnum = 0;
  int count = 0;

  // Instantiate variables for use with the kinematic system.
  double T_pose[4][4], T_des[4][4];
  double q_pose[6], q_des[8][6];
  trajectory_msgs::JointTrajectory desired;

while(ros::ok())
{
  
  //order process
  if (order_vector.size() > 0)
{
    
    //process one product
    std::string type = order_vector[0].shipments[shipmentnum].products[productnum].type;

    osrf_gear::GetMaterialLocations LocationsInfo;
    LocationsInfo.request.material_type = type;
    std::string bins;
    int bin;
    std::string frame;
    // Create variables
    geometry_msgs::PoseStamped part_pose, goal_pose;

    if (materialLocations.call(LocationsInfo)) 
    {
        bins = LocationsInfo.response.storage_units[0].unit_id;
        if (bins == "bin1")
          {
             bin = 2;
             frame = "logical_camera_bin1_frame";
          }
        else if (bins == "bin2")
          {
             bin = 3;
             frame = "logical_camera_bin2_frame";
          }
        else if (bins == "bin3")
          {
             bin = 4;
             frame = "logical_camera_bin3_frame";
          }
        else if (bins == "bin4")
          {
             bin = 5;
             frame = "logical_camera_bin4_frame";
          }
        else if (bins == "bin5")
          {
             bin = 6;
             frame = "logical_camera_bin5_frame";
          }
        else if (bins == "bin6")
          {
             bin = 7;
             frame = "logical_camera_bin6_frame";
          }

        osrf_gear::LogicalCameraImage camInfotem = camInfo [bin];

        geometry_msgs::TransformStamped tfStamped;
        arm_trans(frame, tfStamped);

        // Copy pose from the logical camera.
        part_pose.pose = camInfotem.models[0].pose;
        goal_pose = Set_goal_pose();
       
        tf2::doTransform(part_pose, goal_pose, tfStamped);
    }

    ROS_INFO_STREAM("Product type is" << type);
    ROS_INFO_STREAM("Product unit is" << bins);
    ROS_INFO_STREAM("Product pose is" << part_pose.pose);

    ik_service::PoseIK pose_ik;  
    pose_ik.request.part_pose = goal_pose.pose;
    // Call the Service
    if (ik_pose.call(pose_ik))
       {
          ROS_INFO("Response returned %i solutions", pose_ik.response.num_sols);
          move_arm_to(count, joint_current, pose_ik.response, follow_joint_trajectory);
       }
    else
       {
          ROS_ERROR("Failed to call service ik_service");
       }


    // Another product
    if (productnum + 1 < sizeof(order_vector[0].shipments[productnum].products))
    {
        productnum ++;
    }
    else
    {
        productnum = 0;
        // Another shipment
        if (shipmentnum + 1 < sizeof(order_vector[0].shipments))
        {
            shipmentnum++;
        }
        else
        {
            ROS_INFO("End of order");
            shipmentnum = 0;
            order_vector.erase(order_vector.begin());
        }
    }
}

//current joint info
  ROS_INFO_STREAM_THROTTLE(10, "Current joint states of arm1:" << joint_current << "time = " << ros::Time::now());

loop_rate.sleep();
}

  return 0;
}
