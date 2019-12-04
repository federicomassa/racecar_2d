from racecar_2d import *
from ddpg import *
import numpy

sim = Sim2D(render=True)
sim.frequency = 25
sim.set_track('track.json')

sim.add_player('Acura_NSX_red.png', 4.4, forward_unicycle_model, [0.0,0.0,0.0,0.0])
rand_index = np.random.randint(0, len(sim.race_line))

ddpg = DDPG(6, 2)

num_episodes = 10
episode_length = 1000

a_min = -1.0
a_max = 1.0
omega_min = -1
omega_max = 1

goal_threshold = 1.0

# Every how many episodes save network weights
save_every = 5
save_id = 'test_car'

# Every how many iterations train networks
train_every = 10


save = False
if save_every > 0:
    save = True


while not sim.done:

    for episode in range(num_episodes):
        if save and episode % save_every == 0:
            ddpg.save(save_id)

        # Reset in random point along the racing line
        #rand_index = np.random.randint(0, len(sim.race_line))
        rand_index = 0
        sim.reset(0, rand_index)

        goal_index = rand_index + np.random.randint(0, 30)
        if goal_index >= len(sim.race_line):
            goal_index = goal_index - len(sim.race_line)
        if goal_index < 0:
            goal_index = len(sim.race_line) + goal_index

        goal = sim.race_line[goal_index]
        sim.draw_point(goal, size=5, persistent=True)

        for iteration in range(episode_length):
            if iteration % train_every == 0:
                ddpg.train()

            state = np.array([sim.players[0].current_state[0], \
                sim.players[0].current_state[1], \
                sim.players[0].current_state[2], \
                sim.players[0].current_state[3], \
                goal[0], \
                goal[1]]).reshape((-1, 6))

            action = ddpg.act(state, True).reshape((2,))
    
            # Actions are [-1,1] --> Renormalize
            action_norm = [(action[i] + 1)/2.0 for i in range(action.shape[0])]
            
            controls = (a_min + (a_max - a_min)*action_norm[0], omega_min + (omega_max - omega_min)*action_norm[1])
            sim.update_player(0, controls)

    
    # first_player_state = sim.players[0].current_state 
    # print("x: {:.2f}, y: {:.2f}, theta: {:.2f}, v: {:.2f}".format(first_player_state[0], first_player_state[1], first_player_state[2], first_player_state[3]))

            sim.tick()

            new_v_state = sim.players[0].current_state
            next_state = np.array([new_v_state[0], new_v_state[1], new_v_state[2], new_v_state[3], goal[0], goal[1]]).reshape((1, 6))

            dist_to_goal = np.sqrt(np.square(next_state[0][0] - goal[0]) + np.square(next_state[0][1] - goal[1]))

            end_of_episode = False
            if dist_to_goal < goal_threshold:
                reward = 10000
                print("REACHED GOAL!!!!")
                end_of_episode = True    
            elif not sim.is_inside_track([next_state[0][0], next_state[0][1]]):
                reward = -10000
                end_of_episode = True
            else:
                reward = -1.0

            ddpg.memorize(state, action, next_state, reward, end_of_episode)
            if end_of_episode:
                break