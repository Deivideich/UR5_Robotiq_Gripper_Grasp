#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
const double tau = 2 * M_PI;

void close_gripper(moveit::planning_interface::MoveGroupInterface& move_gripper)
{
    move_gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.220);
    move_gripper.move();
}

void open_gripper(moveit::planning_interface::MoveGroupInterface& move_gripper)
{
    move_gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.0);
    move_gripper.move();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(2.0);

    moveit::planning_interface::MoveGroupInterface group("manipulator");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    // Target position
    geometry_msgs::Pose target_pose1;
    tf2::Quaternion orientation;
    //orientation.setRPY(-tau/4, - tau/4, 0);
    orientation.setRPY(2.759, -1.492, -2.694);
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = -0.705;
    target_pose1.position.y = -0.196;
    target_pose1.position.z = -0.230;
    group.setPoseTarget(target_pose1);

    // Target position 2
    geometry_msgs::Pose target_pose2;
    //orientation.setRPY(-tau/4, - tau/4, 0);
    target_pose2.orientation = tf2::toMsg(orientation);
    target_pose2.position.x = -0.705;
    target_pose2.position.y = -0.196;
    target_pose2.position.z = -0.112;


    // move the group arm
    ros::WallDuration(1.0).sleep();
    group.move();

    ros::WallDuration(1.0).sleep();
    close_gripper(gripper);

    group.setPoseTarget(target_pose2);
    
    ros::WallDuration(1.0).sleep();
    group.move();

    group.setPoseTarget(target_pose1);

    // move the group arm
    ros::WallDuration(1.0).sleep();
    group.move();
    

    ros::WallDuration(1.0).sleep();
    open_gripper(gripper);

    ros::shutdown();
    return 0;

}