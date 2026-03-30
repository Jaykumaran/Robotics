import gymnasium as gym
from gymnasium import spaces
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class CarGoalEnv(gym.Env):
    def __init__(self):
        super(CarGoalEnv, self).__init__()

        # actions: [Linear velocity, Angular Velocity (Turning)]
        self.action_space = spaces.Box(low= np.array([-1.0, -1.0]), high = np.array([1.0, 1.0]),
                                        dtype = np.float32)
        
        # observation: [car_x, car_y, cos(theta), sin(theta), goal_x, goal_y]
        self.observation_space = spaces.Box(low = -np.inf, high=np.inf, shape = (6,), dtype = np.float32)

        self.state = None
        self.goal = np.array([8.0, 8.0])
        self.max_steps = 200
        self.current_step = 0

    def reset(self, seed = None, options = None):
        super().reset(seed = seed)
        # start car at random position near origin
        pos = self.np_random.uniform(low = -2, high = 2, size = (2, ))
        theta = self.np_random.uniform(low = 0, high = 2 * np.pi)
        self.state = np.array([pos[0], pos[1], theta], dtype = np.float32)
        self.current_step = 0
        return self._get_obs(), {}
    
    def _get_obs(self):
        x,y, theta = self.state
        return np.array([x,y, np.cos(theta), np.sin(theta), self.goal[0], self.goal[1]],
                         dtype = np.float32)
    
    
    def step(self, action):
        v, w = action # v = speed, w = steering/rotation rate
        x,y,theta = self.state

        # kinematic
        dt = 0.1
        theta += w * 0.5 # rotate
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt

        self.state = np.array([x,y, theta], dtype=np.float32)
        self.current_step  += 1

        # reward : negative distance to goal
        dist = np.linalg.norm(self.state[:2] - self.goal)

        reward = -dist
        if dist < 0.5: reward += 10 # reaching goal bonus

        terminated = dist < 0.5
        truncated = self.current_step >= self.max_steps

        return self._get_obs(), reward, terminated, truncated, {}
    
    def render(self):
        plt.cla()
        ax = plt.gca()
        x,y, theta = self.state

        # draw goal
        circle = patches.Circle((self.goal[0], self.goal[1]), 0.5, color = 'green', alpha = 0.5)
        ax.add_patch(circle)


        # draw rect
        rect = patches.Rectangle((x-0.2, y-0.1), 0.4, 0.2, angle = np.degrees(theta))

        ax.add_patch(rect)

        # draw heading arrow
        ax.arrow(x,y, 0.5*np.cos(theta), 0.5*np.sin(theta), head_width=0.1, color = 'red')

        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        plt.pause(0.1)