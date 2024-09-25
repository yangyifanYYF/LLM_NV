import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np

# Actor网络
class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(Actor, self).__init__()
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, action_dim)

    def forward(self, state):
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        x = torch.softmax(self.fc3(x), dim=-1)
        return x

# Critic网络
class Critic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(Critic, self).__init__()
        self.fc1 = nn.Linear(state_dim + action_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, 1)

    def forward(self, state, action):
        x = torch.cat([state, action], dim=-1)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x

# Soft Actor-Critic算法
class SAC:
    def __init__(self, state_dim, action_dim, lr=3e-4, gamma=0.99, alpha=0.2):
        self.actor = Actor(state_dim, action_dim)
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr)

        self.critic1 = Critic(state_dim, action_dim)
        self.critic1_optimizer = optim.Adam(self.critic1.parameters(), lr=lr)

        self.critic2 = Critic(state_dim, action_dim)
        self.critic2_optimizer = optim.Adam(self.critic2.parameters(), lr=lr)

        self.target_entropy = -action_dim
        self.log_alpha = torch.zeros(1, requires_grad=True)
        self.alpha_optimizer = optim.Adam([self.log_alpha], lr=lr)

        self.state_dim = state_dim
        self.action_dim = action_dim
        self.gamma = gamma
        self.alpha = alpha

    def select_action(self, state):
        state = torch.FloatTensor(state).unsqueeze(0)
        action_probs = self.actor(state).squeeze(0)
        action = torch.multinomial(action_probs, 1)
        return action.item()

    def update(self, state, action, reward, next_state, done):
        state = torch.FloatTensor(state)
        action = torch.LongTensor(action).unsqueeze(-1)
        reward = torch.FloatTensor(reward).unsqueeze(-1)
        next_state = torch.FloatTensor(next_state)

        # 计算Q值
        q1 = self.critic1(state, action)
        q2 = self.critic2(state, action)

        with torch.no_grad():
            next_action_probs = self.actor(next_state)
            next_action_log_probs = torch.log(next_action_probs + 1e-6)
            next_action_entropy = -(next_action_probs * next_action_log_probs).sum(dim=-1)
            next_q1 = self.critic1(next_state, torch.argmax(next_action_probs, dim=-1, keepdim=True))
            next_q2 = self.critic2(next_state, torch.argmax(next_action_probs, dim=-1, keepdim=True))
            min_next_q = torch.min(next_q1, next_q2) - self.alpha * next_action_entropy
            target_q = reward + self.gamma * (1 - done) * min_next_q

        # Critic网络更新
        critic1_loss = F.mse_loss(q1, target_q)
        critic2_loss = F.mse_loss(q2, target_q)
        self.critic1_optimizer.zero_grad()
        critic1_loss.backward()
        self.critic1_optimizer.step()
        self.critic2_optimizer.zero_grad()
        critic2_loss.backward()
        self.critic2_optimizer.step()

        # Actor网络更新
        action_probs = self.actor(state)
        action_log_probs = torch.log(action_probs + 1e-6)
        actor_loss = (self.alpha * action_log_probs).mean()
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # Alpha更新
        alpha_loss = -(self.log_alpha * (action_log_probs + self.target_entropy).detach()).mean()
        self.alpha_optimizer.zero_grad()
        alpha_loss.backward()
        self.alpha_optimizer.step()
        self.alpha = self.log_alpha.exp().detach()

        return critic1_loss.item(), critic2_loss.item(), actor_loss.item(), alpha_loss.item()
