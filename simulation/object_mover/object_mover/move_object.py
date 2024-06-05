import rclpy
from rclpy.node import Node
import transforms3d

import geometry_msgs.msg
from gazebo_msgs.srv import SetEntityState
import subprocess
import random


class MoveObject(Node):

    def __init__(self):
        super().__init__("move_object")

        self.cli = self.create_client(SetEntityState, "gazebo/set_entity_state")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.get_logger().info("service available")

        # set some default values for the request
        self.req = SetEntityState.Request()
        self.req.state.reference_frame = "world"
        self.req.state.name = "eur_pallet"
        self.pose = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 0.0]

        # create lists to loop through with objects, poses and orientations
        self.orientations = 225  # Request from Jesper: 225 orientations in each cell
        self.grid_x = [22.6, 25.6]  # [22.0, 25.6]
        self.grid_y = [-1, 9]  # [-0.5, 3.8]
        self.grid_count = 15  # Request from Jesper: 15x15 grid

        self.objects = ["jackal", "infobot", "eur_pallet", "box_group_pickup"]
        self.objects_default_position = {
            "jackal": (40.0, 40.0),
            "infobot": (43.0, 43.0),
            "eur_pallet": (46.0, 46.0),
            "box_group_pickup": (50.0, 50.0),
        }
        self.timer_period = 1  # seconds

    def start(self):
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def send_request(self):

        # add new position and orientation to the request
        self.req.state.pose.position.x = self.pose[0]
        self.req.state.pose.position.y = self.pose[1]
        self.req.state.pose.position.z = self.pose[2]
        self.req.state.pose.orientation.x = self.orientation[0]
        self.req.state.pose.orientation.y = self.orientation[1]
        self.req.state.pose.orientation.z = self.orientation[2]
        self.req.state.pose.orientation.w = self.orientation[3]

        future = self.cli.call_async(self.req)

        if future.done():
            self._logger.info("Asynchronous call finished")
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().error("Service call failed %r" % (e,))
            else:
                self._logger.info("Got response: {}".format(response.value))
        return future.result()

    def timer_callback(self):
        self.reset_pose()
        self.send_request()
        self.update_pose()
        self.send_request()

    def reset_pose(self):
        self.pose = [
            self.objects_default_position[self.req.state.name][0],
            self.objects_default_position[self.req.state.name][1],
            0.0,
        ]
        self.orientation = quaternion_from_euler(0, 0, 0)

    def update_pose(self):
        self.req.state.name = random.choice(self.objects)
        self.pose = [
            random.uniform(self.grid_x[0], self.grid_x[1]),
            random.uniform(self.grid_y[0], self.grid_y[1]),
            0.0,
        ]
        self.orientation = quaternion_from_euler(0, 0, random.uniform(-1, 1))


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> list[float]:
    w, x, y, z = transforms3d.euler.euler2quat(roll, pitch, yaw, "rxyz")
    return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()

    move_object = MoveObject()
    move_object.start()

    executor.add_node(move_object)
    executor.spin()
    executor.shutdown()
    move_object.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
