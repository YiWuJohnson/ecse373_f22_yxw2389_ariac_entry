# ecse373_f22_yxw2389_ariac_entry
###use `roslaunch ariac_entry entry.launch` to launch it.
project is based on the information available from the ARIAC website, https://bitbucket.org/osrf/ariac/wiki/2019/documentation

launch files:
ecse_373_ariac.launch
The ecse_373_ariac.launch file is to start the environment and ables the start of competition, it contains the needed information about using pockage ecse_373_ariac.
entry.launch
This launch file runs the ecse_373_ariac.launch that also starts the competition.

Main function:
The functionality will be to move the manipulator over each part in a bin and back using a call to the Action Server interface.

We first create a Trigger object, then call it using a ServiceClient. if the serivce is started successfully, you  can see a ROS info saying:
ROS_INFO("Competition service called successfully
But if it failed, it will be following information.
ROS_ERROR("Competition service call failed! Goodness Gracious!!");
ROS_WARN("Competition service returned failure:  xxx ");
the integer, service_call_succeeded is used to check whether this call is successful.

The competition has started, the first order is published. 
We subscribe to the ariac/order topic to receive this message. order_vector is cleared in main loop to store the order information.
we add orders to the end of the vector using "order_vector.push_back(*msg)"
The order vector can only be searched if an order exists. In the main loop we have a justify "order_vector.size() > 0" to ensure it works when there are some orders.
We use  the size of shipment and product vector to determain when we can finish shipment and move to next order.

The Service GetMaterialLocations is run to find the locations where a product type is. And according to the type information in product vector, we can find its pose.
 It works similarly to Trigger, expect the topic we service is /ariac/material_locations.The locations of the products are found using the 10 logical cameras. 
One for each of the six bins, two for the agvs, and two for quality. We used 10 callback to get their information.

Joint trajectories define how the robot moves. A trajetory conists of seven positions per point.
 Each point is a location the arm moves towards a destination, making them waypoints. 
Each point beyond the initial point can be found using T matrixes.
it have one degree more than the return of ur_kinematics.
We  use inverse(ur_kinematics::inverse((double *)&T, (double *)&q);. 
This method uses a provided T-matrix and finds how the six joints of the UR10 robot arm can angle themselves to have the final joint
the "end effector" reach the desired location. 


