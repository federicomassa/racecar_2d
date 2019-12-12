from racecar_2d import *
from rl_ddpg import *
import numpy

sim = Sim2D(render=False)
sim.frequency = 25
sim.set_track('track.json')

sim.add_player('Acura_NSX_red.png', 4.4, forward_unicycle_model, [0.0,0.0,0.0,0.0])
player = sim.players[0]
player.add_sensor('laser', SensorLaser(player, sim, (-np.pi/6.0, 0.0, np.pi/6.0), 20.0, 0.1))
rand_index = np.random.randint(0, len(sim.race_line))

# states: laser1, laser2, laser3, ds, d, v
# action: acceleration, steering
ddpg = DDPG(6, 2, noise_model='ornstein_uhlenbeck')

num_episodes = 1000
episode_length = 200

a_min = -0.5
a_max = 4.0
omega_min = -1
omega_max = 1

normal_reward = -1
goal_reward = 10000
collision_reward = -10000
goal_threshold = 1.0

# Every how many episodes save network weights
save_every = 5
save_id = 'laser_test_1'

# Every how many iterations train networks
train_every = 10

save = False
if save_every > 0:
    save = True

# Every how many episodes to render
render_every = 5

if ddpg.save_exists(save_id):
    print("Loading from files with prefix: {}".format(save_id))
    ddpg.load(save_id)


for episode in range(num_episodes):
    print ("Starting episode {} with exploration ratio: {}".format(episode, ddpg.epsilon))
    if save and episode % save_every == 0:
        ddpg.save(save_id)

    rendering = False
    if episode % render_every == 0:
        rendering = True
        sim.display_on()
    else:
        sim.display_off(False)

    is_training = not rendering

    # Reset in random point along the racing line
    #rand_index = np.random.randint(0, len(sim.race_line))
    rand_index = 0
    sim.reset(0, rand_index)

    goal_index = rand_index + np.random.randint(0, 50)
    if goal_index >= len(sim.race_line):
        goal_index = goal_index - len(sim.race_line)
    if goal_index < 0:
        goal_index = len(sim.race_line) + goal_index

    goal = sim.race_line[goal_index]
    sim.draw_point(goal, size=5, persistent=True)
    [init_s, _] = sim.get_track_coordinates(player.current_state[0:2])
    [goal_s, _] = sim.get_track_coordinates(goal)
    
    tot_reward = 0

    inside, index_hint = sim.is_inside_track(player.current_state[0:2])
    if not inside:
        raise Exception("Player is not inside track at the beginning of episode {}".format(episode))

    new_readings = None
    for iteration in range(episode_length):

        if is_training and iteration % train_every == 0:
            ddpg.train()

        inside, index_hint = sim.is_inside_track(player.current_state[0:2], index_hint, 20)

        if new_readings == None:
            readings = player.simulate('laser', index_hint=index_hint, interval=20)
        else:
            readings = new_readings

        [s, d] = sim.get_track_coordinates(player.current_state[0:2])
        ds = sim.get_forward_ds(s, goal_s)
        state = np.array([readings[0][1], readings[1][1], readings[2][1], ds, d, player.current_state[3]]).reshape((-1, 6))

        action = ddpg.act(state, is_training).reshape((2,))

        # Actions are [-1,1] --> Renormalize
        action_norm = [(action[i] + 1)/2.0 for i in range(action.shape[0])]
        
        controls = (a_min + (a_max - a_min)*action_norm[0], omega_min + (omega_max - omega_min)*action_norm[1])
        sim.update_player(0, controls)


# first_player_state = player.current_state 
# print("x: {:.2f}, y: {:.2f}, theta: {:.2f}, v: {:.2f}".format(first_player_state[0], first_player_state[1], first_player_state[2], first_player_state[3]))

        sim.tick()

        inside, _ = sim.is_inside_track(player.current_state[0:2], index_hint, 20)
        new_readings = player.simulate('laser', index_hint=index_hint, interval=20)
        [s, d] = sim.get_track_coordinates(player.current_state[0:2])
        ds = sim.get_forward_ds(s, goal_s)
        next_state = np.array([new_readings[0][1], new_readings[1][1], new_readings[2][1], ds, d, player.current_state[3]]).reshape((-1, 6))

        end_of_episode = False
        if ds < goal_threshold:
            reward = goal_reward
            print("REACHED GOAL!!!!")
            end_of_episode = True    
        elif not inside:
            reward = collision_reward
            end_of_episode = True
        else:
            reward = normal_reward

        tot_reward += reward

        if is_training:
            ddpg.memorize(state, action, next_state, reward, end_of_episode)
        

        if end_of_episode or iteration == episode_length - 1:
            print("Init goal distance: {}, Total reward: {}".format(sim.get_forward_ds(init_s, goal_s), tot_reward))
            break