#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import random
import math
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose

class game(Node):
    def __init__(self):
        super().__init__("publisher")
        self.enemy_positions = {}
        self.player_pose = None
        self.score = 0
        self.get_logger().info("Game start!")
        self.create_subscription(Pose, "turtle1/pose", self.player_callback, 10)
        for i in range(1,4):
            self.spawn_enemies(f"enemy{i}")
        self.create_timer(0.1, self.check_collision)
        self.score_publisher = self.create_publisher(Int32, "/score", 10)

    def spawn_enemies(self, name):
        client=self.create_client(Spawn,"spawn")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for creating new enemy ...")
        request=Spawn.Request()
        request.x=random.uniform(1.0, 10.0)
        request.y=random.uniform(1.0,10.0)
        request.theta= 0.0
        request.name=name
        future=client.call_async(request)
        future.add_done_callback(lambda future: self.spawn_enemies_callback(future, name))
 
    
    def spawn_enemies_callback(self, future, name):
        try:
            enemy=future.result()
            self.get_logger().info(f"Add new enemy: {enemy.name}")
            self.create_subscription(Pose, f"{name}/pose", lambda msg, n=name: self.enemy_position(msg, n), 10)
        except Exception as e:
            self.get_logger().error("Service call failed: %r" %(e,))
    
    def player_callback(self, msg):
        self.player_pose = msg

    def enemy_position(self, msg, name):
        self.enemy_positions[name] = msg

    def check_collision(self):
        if self.player_pose is None:
            return
        for name, pose in list(self.enemy_positions.items()):
            distance = math.sqrt((self.player_pose.y - pose.y)**2 + (self.player_pose.x - pose.x)**2)
            if distance < 0.5:
                self.get_logger().info(f"{name} is hit")
                self.kill_enemy(name)
                del self.enemy_positions[name]
                self.score += 1
                score_msg = Int32()
                score_msg.data = self.score
                self.score_publisher.publish(score_msg)
                

    def kill_enemy(self, name):
        client=self.create_client(Kill,"kill")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for deleting enemy ...")
        request=Kill.Request()
        request.name=name
        future=client.call_async(request)
        future.add_done_callback(lambda future: self.kill_enemy_callback(future, name))
    def kill_enemy_callback(self, future, name):
        try:
            future.result()
            self.get_logger().info(f"{name} killed!")
            self.spawn_enemies(name)
        except Exception as e:
            self.get_logger().error("Service call failed: %r" %(e,))
    
def main():
    rclpy.init()
    node = game()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    
                        


