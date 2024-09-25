from sac import SAC


def run(decision, start_point, destination_point, walkable_points, visible_obstacles):
    # 环境设置
    state_dim = 5
    action_dim = 1

    # 创建SAC对象
    sac = SAC(state_dim, action_dim)

    # 训练
    max_episodes = 100
    max_steps = 500
    for episode in range(max_episodes):
        state = env.reset()
        episode_reward = 0
        for t in range(max_steps):
            action = sac.select_action(state)
            next_state, reward, done, _ = env.step(action)
            sac.update(state, action, reward, next_state, done)
            state = next_state
            episode_reward += reward
            if done:
                break
        print(f"Episode: {episode + 1}, Reward: {episode_reward}")

    # 关闭环境
    env.close()