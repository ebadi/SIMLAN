from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.duration import Duration

# If pos commands are not received by AMCL and this errors is shown: "AMCL covariance or pose is NaN, likely due to an invalid configuration or faulty sensor measurements! Pose is not available!"
# Make sure that nav2 stack is up before infobot

# Inspired by https://github.com/ros-planning/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/example_nav_to_pose.py


def main(args=None):
    rclpy.init()
    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"

    # INITIAL_POSITION
    initial_pose.pose.position.x = 3.0
    initial_pose.pose.position.y = 7.0
    initial_pose.pose.position.z = 0.5
    # initial_pose.pose.orientation.z = 0.0
    # initial_pose.pose.orientation.w = 3.14/2
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active(localizer="bt_navigator")

    navigator.changeMap("maps/warehouse.yaml")

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"

    goal_pose.pose.position.x = 20.0
    goal_pose.pose.position.y = 15.0
    goal_pose.pose.orientation.w = 0.0
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(goal_pose)
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:

            global_costmap = navigator.getGlobalCostmap()
            local_costmap = navigator.getLocalCostmap()

            print(global_costmap, local_costmap)
            print(
                "Estimated time of arrival: "
                + "{0:.0f}".format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + " seconds."
            )


if __name__ == "__main__":
    main()
