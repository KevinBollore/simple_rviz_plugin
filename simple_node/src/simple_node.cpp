// Standard headers
#include <string>

// ROS headers
#include <ros/ros.h>
#include <ros/service.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include <simple_node/SendOffset.h> // Description of the Service we will use

boost::shared_ptr<move_group_interface::MoveGroup> group;
boost::shared_ptr<ros::NodeHandle> node;

/**
 * This is the service function that is called whenever a request is received
 * @param req[int]
 * @param res[out]
 * @return Alway true
 */
bool moveRobot(simple_node::SendOffset::Request &req, simple_node::SendOffset::Response &res)
{
  // Get parameters from the message and print them
  ROS_WARN_STREAM("moveRobot request:" << std::endl << req);

  // Get current robot pose
  tf::TransformListener listener;
  listener.waitForTransform("/base", "/tool0", ros::Time::now(), ros::Duration(1.0));
  tf::StampedTransform transform_stamp;
  Eigen::Affine3d current_pose;
  try
  {
    listener.lookupTransform("/base", "/tool0", ros::Time(0), transform_stamp);
    transformTFToEigen(transform_stamp, current_pose);
  }
  catch (tf::TransformException &ex)
  {
    res.ReturnMessage = ex.what();
    res.ReturnStatus = false;
    return true;
  }

  // Apply the offset on the given axis of the current pose
  if(req.Axis == "x")
  {
    current_pose.translate(Eigen::Vector3d(req.Offset / 1000.0, 0, 0));
  }
  else if(req.Axis == "y")
  {
    current_pose.translate(Eigen::Vector3d(0, req.Offset / 1000.0, 0));
  }
  else if(req.Axis == "z")
  {
    current_pose.translate(Eigen::Vector3d(0, 0, req.Offset / 1000.0));
  }
  else
  {
    ROS_ERROR_STREAM("Problem occurred during axis request. Aborting...");
    res.ReturnStatus = false;
    res.ReturnMessage = "Problem occurred during axis request";
    return true;
  }

  // Try to move to new pose
  std::vector<geometry_msgs::Pose> way_points_msg(1);
  tf::poseEigenToMsg(current_pose, way_points_msg[0]);

  moveit_msgs::ExecuteKnownTrajectory srv;
  srv.request.wait_for_execution = true;
  ros::ServiceClient executeKnownTrajectoryServiceClient = node->serviceClient<moveit_msgs::ExecuteKnownTrajectory>(
      "/execute_kinematic_path");
  if (group->computeCartesianPath(way_points_msg, 0.05, 0, srv.request.trajectory) < 0.99)
  {
    // No trajectory can be found, aborting and sending error message:
    res.ReturnStatus = false;
    res.ReturnMessage = "Cannot reach pose!";
    return true;
  }

  executeKnownTrajectoryServiceClient.call(srv);
  res.ReturnStatus = true;
  res.ReturnMessage = "Robot moved successfully";
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_robot");
  node.reset(new ros::NodeHandle);

  // Initialize move group
  group.reset(new move_group_interface::MoveGroup("manipulator"));
  group->setPoseReferenceFrame("/base");
  group->setPlannerId("RRTConnectkConfigDefault");
  group->setPlanningTime(2);

  // Create service server and wait for incoming requests
  ros::ServiceServer service = node->advertiseService("move_robot_offset", moveRobot);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (node->ok())
  {
  }
  return 0;
}
