#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import torch
import torch.nn as nn
import torch.optim as optim
from math import random
import numpy as np
from sensor_msgs.msg import Imu
from mavros_msgs.msg import ManualControl, Altitude
from std_msgs.msg import Int16

class PVNetwork(nn.Module):
    def __init__(self, input_dim, action_dim):
        super(PVNetwork, self).__init__()
        self.fc1 = nn.Linear(input_dim, 128)
        self.fc2 = nn.Linear(128, 128)
        self.policy_head = nn.Linear(128, action_dim)
        self.value_head = nn.Linear(128, 1)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        policy = torch.softmax(self.policy_head(x), dim=-1)
        value = torch.tanh(self.value_head(x))
        return policy, value

def monte_carlo_tree_search(root, pv_network, simulations=1000, max_iteration=50):
    for _ in range(simulations):
        node = root
        state = node.state

        # Selection and Expansion
        for _ in range(max_iteration):
            if not node.is_fully_expanded():
                policy, value = pv_network(torch.tensor(state.to_array(), dtype=torch.float32).unsqueeze(0))
                action_probs = policy.detach().numpy().flatten()
                best_action = np.argmax(action_probs)
                new_state = state.perform_action(node.state.get_possible_actions()[best_action])
                node = node.expand(new_state, best_action)
                break
            else:
                node = node.best_child()
                state = node.state

        # Simulation
        reward = default_policy(state)

        # Backpropagation
        while node is not None:
            node.update(reward)
            node = node.parent

    return best_child(root, 0)

def default_policy(state):
    while not state.is_terminal():
        action = random.choice(state.get_possible_actions())
        state = state.perform_action(action)
    return state.get_reward()

def best_child(node, c_param):
    choices_weights = [
        (child.value / child.visits) + c_param * np.sqrt((2 * np.log(node.visits) / child.visits))
        for child in node.children
    ]
    return node.children[np.argmax(choices_weights)]

def train_pv_network(pv_network, data, epochs=10, batch_size=32, learning_rate=0.001):
    optimizer = optim.Adam(pv_network.parameters(), lr=learning_rate)
    loss_fn = nn.MSELoss()

    for epoch in range(epochs):
        for i in range(0, len(data), batch_size):
            batch = data[i:i + batch_size]
            states, policies, values = zip(*batch)

            states = torch.tensor(states, dtype=torch.float32)
            policies = torch.tensor(policies, dtype=torch.float32)
            values = torch.tensor(values, dtype=torch.float32)

            optimizer.zero_grad()
            pred_policies, pred_values = pv_network(states)
            loss = loss_fn(pred_policies, policies) + loss_fn(pred_values.squeeze(), values)
            loss.backward()
            optimizer.step()

def collect_data(root, pv_network, simulations=1000):
    data = []
    for _ in range(simulations):
        node = root
        state = node.state
        search_probs = []
        for _ in range(max_iteration):
            policy, value = pv_network(torch.tensor(state.to_array(), dtype=torch.float32).unsqueeze(0))
            action_probs = policy.detach().numpy().flatten()
            search_probs.append(action_probs)
            best_action = np.argmax(action_probs)
            state = state.perform_action(node.state.get_possible_actions()[best_action])
            if state.is_terminal():
                break
        reward = state.get_reward()
        data.append((state.to_array(), search_probs, reward))
    return data

class AUVController(Node):
    def __init__(self, pv_network, *args, **kwargs):
        super().__init__('auv_controller')
        self.camera_subscriber = self.create_subscription(Image, 'camera/image_raw', self.camera_callback, 10)
        self.depth_subscriber = self.create_subscription(Range, 'depth', self.depth_callback, 10)
        self.heading_subscriber = self.create_subscription(Imu, 'imu', self.heading_callback, 10)
        self.manual_control_publisher = self.create_publisher(ManualControl, '/mavros/manual_control/control', 10)
        
        self.pv_network = pv_network
        self.current_state = State.SCANNING
        self.current_heading = None
        self.current_depth = None
        self.distance_to_opponent = None
        self.relative_heading_to_opponent = None
        self.current_game_state = None

    def camera_callback(self, msg):
        # Process image to detect the opponent and update distance and relative heading
        pass

    def depth_callback(self, msg):
        self.current_depth = msg.range

    def heading_callback(self, msg):
        orientation_q = msg.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        self.current_heading = np.degrees(yaw)

    def update_state(self):
        self.current_game_state = GameState(
            heading=self.current_heading,
            depth=self.current_depth,
            distance_to_opponent=self.distance_to_opponent,
            relative_heading_to_opponent=self.relative_heading_to_opponent
        )
        self.transition_state()

    def transition_state(self):
        if self.current_state == State.SCANNING:
            if self.distance_to_opponent is not None:
                self.current_state = State.MOVING
        elif self.current_state == State.MOVING:
            if self.distance_to_opponent < 5.0:
                self.current_state = State.ORBITING
        elif self.current_state == State.ORBITING:
            if self.distance_to_opponent < 2.0:
                self.current_state = State.FLASHING
        elif self.current_state == State.FLASHING:
            if self.distance_to_opponent > 2.0:
                self.current_state = State.ORBITING

    def perform_action(self):
        if self.current_state == State.SCANNING:
            self.scan()
        elif self.current_state == State.MOVING:
            self.run_mcts()
        elif self.current_state == State.ORBITING:
            self.run_mcts()
        elif self.current_state == State.FLASHING:
            self.flash()

    def scan(self):
        cmd = ManualControl()
        cmd.x = 1500
        cmd.y = 1500
        cmd.z = 1500
        cmd.r = 1600
        self.manual_control_publisher.publish(cmd)

    def run_mcts(self):
        if self.current_game_state is not None:
            root = TreeNode(self.current_game_state)
            best_action = monte_carlo_tree_search(root, self.pv_network)
            self.perform_action_with_mcts(best_action)

    def perform_action_with_mcts(self, action):
        delta_heading, delta_depth = action
        cmd = ManualControl()
        cmd.x = 1600
        cmd.y = 1500
        cmd.z = 1500 + delta_depth * 100
        cmd.r = 1500 + delta_heading * 10
        self.manual_control_publisher.publish(cmd)

    def flash(self):
        self.get_logger().info("Flashing light!")

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            self.update_state()
            self.perform_action()

def main(args=None):
    input_dim = 4  # Example input dimension
    action_dim = 9  # 3 possible heading changes (-10, 0, 10) and 3 possible depth changes (-1, 0, 1)
    pv_network = PVNetwork(input_dim, action_dim)

    rclpy.init(args=args)
    controller = AUVController(pv_network)
    controller.run()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

