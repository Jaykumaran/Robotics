# learner
import torch
import torch.optim as optim
from networks import Actor, Critic
import torch.nn.functional as F

class SACAgent:
    def __init__(self, state_dim, action_dim):
        self.gamma = 0.99
        self.tau = 0.005
        self.alpha = 0.2 # entropy coefficient
        self.device = torch.device('cuda' if torch.cuda.is_available() else "cpu")

        self.actor = Actor(state_dim, action_dim).to(self.device)
        self.critic = Critic(state_dim, action_dim).to(self.device)
        self.critic_target = Critic(state_dim, action_dim).to(self.device)
        self.critic_target.load_state_dict(self.critic.state_dict())

        self.actor_opt = optim.Adam(self.actor.parameters(), lr = 3e-4)
        self.critic_opt = optim.Adam(self.critic.parameters(), lr = 3e-4)

    
    def select_action(self, state, evaluate= False):
        state = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        if evaluate:
            mu, _ = self.actor(state)
            return torch.tanh(mu).detach().cpu().numpy()[0]
        
        else:
            action, _ = self.actor.sample(state)
            return action.detach().cpu().numpy()[0] # detach from comp graph
        
    def update_parameters(self, memory, batch_size):
        state, action, reward, next_state, done = memory.sample(batch_size)
        state = torch.FloatTensor(state).to(self.device)
        action = torch.FloatTensor(action).to(self.device)
        reward = torch.FloatTensor(reward).unsqueeze(1).to(self.device)
        next_state = torch.FloatTensor(next_state).to(self.device)
        done = torch.FloatTensor(done).unsqueeze(1).to(self.device)

        with torch.no_grad():
            next_action, next_log_prob = self.actor.sample(next_state)
            q1_next, q2_next = self.critic_target(next_state, next_action)
            min_q_next = torch.min(q1_next, q2_next) - self.alpha * next_log_prob
            target_q = reward + (1 - done) * self.gamma * min_q_next

        # update critic
        q1, q2 = self.critic(state, action)
        critic_loss = F.mse_loss(q1, target_q) + F.mse_loss(q2, target_q)
        self.critic_opt.zero_grad()
        critic_loss.backward()
        self.critic_opt.step()

        # update Actor
        new_action, log_prob = self.actor.sample(state)
        q1_new, q2_new = self.critic(state, new_action)
        actor_loss = (self.alpha * log_prob - torch.min(q1_new, q2_new)).mean()
        self.actor_opt.zero_grad()
        actor_loss.backward()
        self.actor_opt.step()

        # soft update target networks
        for target_param, param in zip(self.critic_target.parameters(), self.critic.parameters()):
            target_param.data.copy_(target_param.data * (1.0 - self.tau) + param.data * self.tau)
        




