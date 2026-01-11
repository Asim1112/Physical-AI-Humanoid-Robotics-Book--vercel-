#!/usr/bin/env python3
# Example: Sim-to-real transfer learning for humanoid robot

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Image
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray, String, Bool
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
import math
import random
import time
from collections import deque
import threading

class SimToRealTransferNode(Node):
    """
    Demonstrates sim-to-real transfer learning concepts for humanoid robots.
    Shows how policies trained in simulation can be adapted for real hardware.
    """

    def __init__(self):
        super().__init__('sim_to_real_transfer_node')

        # Declare parameters
        self.declare_parameter('learning_frequency', 10.0)  # Hz
        self.declare_parameter('policy_update_frequency', 1.0)  # Hz
        self.declare_parameter('batch_size', 32)
        self.declare_parameter('learning_rate', 1e-4)
        self.declare_parameter('gamma', 0.99)  # Discount factor
        self.declare_parameter('tau', 0.005)   # Soft update parameter
        self.declare_parameter('buffer_size', 10000)
        self.declare_parameter('sim_noise_level', 0.1)  # Simulation noise
        self.declare_parameter('real_noise_level', 0.3)  # Real robot noise

        # Get parameters
        self.learning_frequency = self.get_parameter('learning_frequency').value
        self.policy_update_frequency = self.get_parameter('policy_update_frequency').value
        self.batch_size = self.get_parameter('batch_size').value
        self.learning_rate = self.get_parameter('learning_rate').value
        self.gamma = self.get_parameter('gamma').value
        self.tau = self.get_parameter('tau').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.sim_noise_level = self.get_parameter('sim_noise_level').value
        self.real_noise_level = self.get_parameter('real_noise_level').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Robot state
        self.joint_states = None
        self.imu_data = None
        self.camera_data = None
        self.current_pose = None

        # Learning components
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.state_dim = 24  # Example: 12 joint positions + 12 joint velocities
        self.action_dim = 12  # Example: 12 joint position commands

        # Initialize policy networks
        self.actor = self.build_actor_network().to(self.device)
        self.critic = self.build_critic_network().to(self.device)
        self.target_actor = self.build_actor_network().to(self.device)
        self.target_critic = self.build_critic_network().to(self.device)

        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=self.learning_rate)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=self.learning_rate)

        # Experience replay buffer
        self.replay_buffer = deque(maxlen=self.buffer_size)

        # Domain randomization parameters
        self.domain_params = {
            'mass_multiplier': 1.0,
            'friction_coefficient': 0.5,
            'actuator_delay': 0.0,
            'sensor_noise': 0.0
        }

        # Transfer adaptation parameters
        self.sim_to_real_adaptation = {
            'applied': False,
            'adaptation_factor': 0.0,
            'transfer_success_rate': 0.0
        }

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.camera_callback, 10)

        # Publishers
        self.action_pub = self.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 10)
        self.learning_status_pub = self.create_publisher(
            String, '/learning/status', 10)
        self.adaptation_pub = self.create_publisher(
            Float64MultiArray, '/adaptation/parameters', 10)

        # Timers
        self.learning_timer = self.create_timer(
            1.0 / self.learning_frequency, self.learning_step)
        self.policy_update_timer = self.create_timer(
            1.0 / self.policy_update_frequency, self.update_policy)

        # Performance tracking
        self.learning_stats = {
            'episodes': 0,
            'steps': 0,
            'rewards': deque(maxlen=100),
            'losses': deque(maxlen=100),
            'transfer_attempts': 0,
            'transfer_successes': 0
        }

        # Initialize target networks
        self.hard_update(self.target_actor, self.actor)
        self.hard_update(self.target_critic, self.critic)

        self.get_logger().info(
            f'Sim-to-Real Transfer node initialized with state_dim={self.state_dim}, '
            f'action_dim={self.action_dim}'
        )

    def joint_callback(self, msg):
        """Update joint state information."""
        self.joint_states = msg

    def imu_callback(self, msg):
        """Update IMU data."""
        self.imu_data = msg

    def camera_callback(self, msg):
        """Process camera data."""
        try:
            self.camera_data = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {e}')

    def build_actor_network(self):
        """Build actor network for policy."""
        return nn.Sequential(
            nn.Linear(self.state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, self.action_dim),
            nn.Tanh()  # Actions between -1 and 1
        )

    def build_critic_network(self):
        """Build critic network for value estimation."""
        return nn.Sequential(
            nn.Linear(self.state_dim + self.action_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 1)
        )

    def get_state(self):
        """Get current robot state for the policy."""
        if self.joint_states is None:
            # Return default state if no joint data available
            return np.zeros(self.state_dim)

        # Extract joint positions and velocities
        positions = np.array(self.joint_states.position)
        velocities = np.array(self.joint_states.velocity)

        # Ensure we have the right number of joints
        if len(positions) < self.state_dim // 2:
            positions = np.pad(positions, (0, self.state_dim // 2 - len(positions)))
            velocities = np.pad(velocities, (0, self.state_dim // 2 - len(velocities)))
        elif len(positions) > self.state_dim // 2:
            positions = positions[:self.state_dim // 2]
            velocities = velocities[:self.state_dim // 2]

        # Combine positions and velocities
        state = np.concatenate([positions, velocities])

        # Add IMU data if available
        if self.imu_data:
            # Extract orientation and angular velocity
            orientation = [
                self.imu_data.orientation.x,
                self.imu_data.orientation.y,
                self.imu_data.orientation.z,
                self.imu_data.orientation.w
            ]
            angular_vel = [
                self.imu_data.angular_velocity.x,
                self.imu_data.angular_velocity.y,
                self.imu_data.angular_velocity.z
            ]
            # Add to state (simplified)
            state = np.concatenate([state, orientation[:2], angular_vel[:2]])  # Use only some values

        return state

    def select_action(self, state, add_noise=True):
        """Select action using current policy."""
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)

        with torch.no_grad():
            action = self.actor(state_tensor).cpu().numpy()[0]

        # Add exploration noise if requested
        if add_noise:
            noise = np.random.normal(0, 0.1, size=action.shape)
            action = np.clip(action + noise, -1, 1)

        return action

    def calculate_reward(self, state, action, next_state):
        """Calculate reward for the transition."""
        # Simplified reward function for humanoid balance
        # In practice, this would be much more complex

        # Extract joint positions and velocities from states
        joint_pos = next_state[:len(next_state)//2]
        joint_vel = next_state[len(next_state)//2:]

        # Reward for maintaining balance (simplified)
        balance_reward = 0.0

        # If IMU data is available, use it for balance assessment
        if self.imu_data:
            # Extract orientation from IMU (simplified)
            # In practice, you'd convert quaternion to roll/pitch
            balance_reward = 1.0  # Base reward

            # Penalize excessive tilt (simplified)
            if abs(self.imu_data.orientation.x) > 0.3 or abs(self.imu_data.orientation.y) > 0.3:
                balance_reward -= 0.5

        # Reward for smooth movement
        smoothness_penalty = -np.mean(np.abs(joint_vel)) * 0.1

        # Reward for staying near neutral joint positions
        neutral_pos_reward = -np.mean(np.abs(joint_pos)) * 0.05

        total_reward = balance_reward + smoothness_penalty + neutral_pos_reward

        return total_reward

    def learning_step(self):
        """Execute one step of the learning algorithm."""
        # Get current state
        current_state = self.get_state()

        # Select action
        action = self.select_action(current_state)

        # In simulation, we would execute the action and get next state
        # For this example, we'll simulate the next state
        next_state = self.simulate_next_state(current_state, action)

        # Calculate reward
        reward = self.calculate_reward(current_state, action, next_state)

        # Store transition in replay buffer
        self.replay_buffer.append((current_state, action, reward, next_state, False))  # done=False for now

        # Update learning stats
        self.learning_stats['steps'] += 1
        self.learning_stats['rewards'].append(reward)

    def simulate_next_state(self, current_state, action):
        """Simulate next state based on current state and action."""
        # This is a simplified simulation of state transition
        # In practice, this would involve the robot's dynamics model

        # Apply action to current state with some dynamics
        next_state = current_state.copy()

        # Modify joint positions based on action (simplified)
        joint_count = len(current_state) // 2
        joint_positions = next_state[:joint_count]
        joint_velocities = next_state[joint_count:]

        # Apply action as desired position change
        action_influence = 0.1  # How much the action affects the state
        desired_positions = joint_positions + action[:len(joint_positions)] * action_influence

        # Update positions with some dynamics
        next_state[:joint_count] = 0.8 * joint_positions + 0.2 * desired_positions + np.random.normal(0, 0.01, joint_positions.shape)

        # Update velocities based on position changes
        position_changes = next_state[:joint_count] - joint_positions
        next_state[joint_count:] = 0.7 * joint_velocities + 0.3 * position_changes + np.random.normal(0, 0.005, joint_velocities.shape)

        # Add noise to simulate real-world uncertainty
        noise_level = self.real_noise_level if self.sim_to_real_adaptation['applied'] else self.sim_noise_level
        next_state += np.random.normal(0, noise_level, next_state.shape)

        return next_state

    def update_policy(self):
        """Update policy using experience replay."""
        if len(self.replay_buffer) < self.batch_size:
            return

        # Sample batch from replay buffer
        batch_indices = np.random.choice(len(self.replay_buffer), self.batch_size, replace=False)
        batch = [self.replay_buffer[i] for i in batch_indices]

        # Extract batch components
        states = torch.FloatTensor([transition[0] for transition in batch]).to(self.device)
        actions = torch.FloatTensor([transition[1] for transition in batch]).to(self.device)
        rewards = torch.FloatTensor([transition[2] for transition in batch]).to(self.device).unsqueeze(1)
        next_states = torch.FloatTensor([transition[3] for transition in batch]).to(self.device)
        dones = torch.BoolTensor([transition[4] for transition in batch]).to(self.device).unsqueeze(1)

        # Update critic
        with torch.no_grad():
            next_actions = self.target_actor(next_states)
            next_q_values = self.target_critic(torch.cat([next_states, next_actions], dim=1))
            target_q_values = rewards + (self.gamma * next_q_values * ~dones)

        current_q_values = self.critic(torch.cat([states, actions], dim=1))
        critic_loss = nn.MSELoss()(current_q_values, target_q_values)

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # Update actor
        predicted_actions = self.actor(states)
        actor_loss = -self.critic(torch.cat([states, predicted_actions], dim=1)).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # Soft update target networks
        self.soft_update(self.target_actor, self.actor, self.tau)
        self.soft_update(self.target_critic, self.critic, self.tau)

        # Update learning stats
        self.learning_stats['losses'].append(actor_loss.item())

        # Log learning progress periodically
        if self.learning_stats['steps'] % 100 == 0:
            avg_reward = np.mean(self.learning_stats['rewards']) if self.learning_stats['rewards'] else 0
            avg_loss = np.mean(self.learning_stats['losses']) if self.learning_stats['losses'] else 0

            self.get_logger().info(
                f'Learning Stats - Steps: {self.learning_stats["steps"]}, '
                f'Avg Reward: {avg_reward:.3f}, Avg Loss: {avg_loss:.6f}'
            )

    def soft_update(self, target_net, source_net, tau):
        """Soft update of target network from source network."""
        for target_param, param in zip(target_net.parameters(), source_net.parameters()):
            target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)

    def hard_update(self, target_net, source_net):
        """Hard update of target network from source network."""
        for target_param, param in zip(target_net.parameters(), source_net.parameters()):
            target_param.data.copy_(param.data)

    def apply_domain_randomization(self):
        """Apply domain randomization to simulation parameters."""
        # Randomize physical parameters to bridge sim-to-real gap
        self.domain_params['mass_multiplier'] = np.random.uniform(0.8, 1.2)
        self.domain_params['friction_coefficient'] = np.random.uniform(0.3, 0.7)
        self.domain_params['actuator_delay'] = np.random.uniform(0.0, 0.02)
        self.domain_params['sensor_noise'] = np.random.uniform(0.01, 0.05)

        # Publish adaptation parameters
        params_msg = Float64MultiArray()
        params_msg.data = [
            self.domain_params['mass_multiplier'],
            self.domain_params['friction_coefficient'],
            self.domain_params['actuator_delay'],
            self.domain_params['sensor_noise']
        ]
        self.adaptation_pub.publish(params_msg)

    def adapt_policy_for_real_robot(self):
        """Adapt simulation-trained policy for real robot."""
        self.get_logger().info('Starting sim-to-real adaptation...')

        # Apply domain randomization during training
        self.apply_domain_randomization()

        # Adjust policy for real-world characteristics
        # This could involve:
        # 1. Adding noise to training data
        # 2. Adjusting control gains
        # 3. Fine-tuning on real robot data
        # 4. Adversarial domain adaptation

        # Update adaptation parameters
        self.sim_to_real_adaptation['applied'] = True
        self.sim_to_real_adaptation['adaptation_factor'] = 0.8  # 80% adaptation

        # Log adaptation status
        adaptation_msg = String()
        adaptation_msg.data = f'Sim-to-real adaptation applied with factor {self.sim_to_real_adaptation["adaptation_factor"]}'
        self.learning_status_pub.publish(adaptation_msg)

        self.get_logger().info('Sim-to-real adaptation completed')

    def evaluate_transfer_success(self):
        """Evaluate the success of sim-to-real transfer."""
        # In practice, this would run the policy on the real robot
        # and measure performance compared to simulation

        # For this example, we'll simulate the evaluation
        success_rate = 0.7  # Simulated success rate

        self.sim_to_real_adaptation['transfer_success_rate'] = success_rate
        self.learning_stats['transfer_attempts'] += 1
        if success_rate > 0.6:  # If successful
            self.learning_stats['transfer_successes'] += 1

        # Log transfer evaluation
        status_msg = String()
        status_msg.data = f'Transfer evaluation - Success rate: {success_rate:.2f}, ' \
                         f'Attempts: {self.learning_stats["transfer_attempts"]}, ' \
                         f'Successes: {self.learning_stats["transfer_successes"]}'
        self.learning_status_pub.publish(status_msg)

        return success_rate

    def get_learning_status(self):
        """Get current learning and adaptation status."""
        status = {
            'episodes': self.learning_stats['episodes'],
            'steps': self.learning_stats['steps'],
            'avg_reward': np.mean(self.learning_stats['rewards']) if self.learning_stats['rewards'] else 0,
            'transfer_applied': self.sim_to_real_adaptation['applied'],
            'adaptation_factor': self.sim_to_real_adaptation['adaptation_factor'],
            'transfer_success_rate': self.sim_to_real_adaptation['transfer_success_rate'],
            'buffer_size': len(self.replay_buffer),
            'replay_capacity': self.buffer_size
        }
        return status

    def execute_policy_on_robot(self, action):
        """Execute the learned policy action on the real robot."""
        # Convert action to joint commands
        cmd_msg = Float64MultiArray()

        # Scale action to appropriate joint command range
        # In practice, this would involve inverse kinematics, trajectory generation, etc.
        scaled_action = action * 0.5  # Scale to reasonable joint command range
        cmd_msg.data = scaled_action.tolist()

        # Publish action to robot
        self.action_pub.publish(cmd_msg)

        return cmd_msg.data


class CurriculumLearning:
    """Implements curriculum learning for sim-to-real transfer."""

    def __init__(self):
        self.stages = [
            {'name': 'balance_training', 'difficulty': 0.1, 'success_threshold': 0.8},
            {'name': 'simple_movement', 'difficulty': 0.3, 'success_threshold': 0.7},
            {'name': 'complex_locomotion', 'difficulty': 0.5, 'success_threshold': 0.6},
            {'name': 'real_world_task', 'difficulty': 0.8, 'success_threshold': 0.5}
        ]
        self.current_stage = 0
        self.stage_performance = [0.0] * len(self.stages)

    def advance_curriculum(self, current_performance):
        """Advance curriculum based on performance."""
        current_stage_info = self.stages[self.current_stage]

        if current_performance >= current_stage_info['success_threshold']:
            if self.current_stage < len(self.stages) - 1:
                self.current_stage += 1
                return True  # Advanced to next stage
        return False  # Did not advance

    def get_current_task(self):
        """Get the current learning task."""
        return self.stages[self.current_stage]['name']

    def modify_training_conditions(self, sim_to_real_node):
        """Modify training conditions based on curriculum stage."""
        current_stage = self.stages[self.current_stage]

        if current_stage['name'] == 'balance_training':
            # Focus on balance maintenance
            sim_to_real_node.apply_domain_randomization()
        elif current_stage['name'] == 'simple_movement':
            # Add simple movement tasks
            pass
        elif current_stage['name'] == 'complex_locomotion':
            # Add complex movement patterns
            pass
        elif current_stage['name'] == 'real_world_task':
            # Prepare for real-world deployment
            sim_to_real_node.adapt_policy_for_real_robot()


def main(args=None):
    """Main function to run the Sim-to-Real Transfer node."""
    rclpy.init(args=args)

    sim_to_real_node = SimToRealTransferNode()

    # Example: Start curriculum learning
    curriculum = CurriculumLearning()

    try:
        sim_to_real_node.get_logger().info('Starting Sim-to-Real Transfer node...')

        # Periodically evaluate and advance curriculum
        curriculum_timer = sim_to_real_node.create_timer(5.0, lambda: None)  # Placeholder

        rclpy.spin(sim_to_real_node)
    except KeyboardInterrupt:
        sim_to_real_node.get_logger().info('Sim-to-Real Transfer node interrupted by user')
    finally:
        sim_to_real_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()