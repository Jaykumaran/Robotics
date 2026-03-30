from env import CarGoalEnv
from agent import SACAgent
from buffer import ReplayBuffer
import torch

env = CarGoalEnv()
agent = SACAgent(6, 2)
memory = ReplayBuffer(100000)
batch_size = 128
episodes = 5000

for ep in range(episodes):
    state, _ = env.reset()
    episode_reward = 0

    for step in range(200):
        action = agent.select_action(state)
        next_state, reward, done, truncated, _ = env.step(action)

        memory.push(state, action, reward, next_state, done)

        if len(memory) > batch_size:
            agent.update_parameters(memory, batch_size)
        
        state = next_state
        episode_reward += reward

        if ep % 50 == 0: # vis every 50 episodes

            env.render()
        if done or truncated:
            break
    
    print(f"Episode: {ep}, Reward: {episode_reward:.2f}")

env.close()
        
