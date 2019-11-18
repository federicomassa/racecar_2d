from sim2d import Sim2D
from sim2d import unicycle_model
import numpy

sim = Sim2D()
sim.frequency = 25
sim.set_track('track.json')

init_pose = []
init_pose.append((sim.race_line[0][0], sim.race_line[0][1], 0.0))
init_pose.append((sim.race_line[10][0], sim.race_line[10][1], 0.0))
init_pose.append((sim.race_line[20][0], sim.race_line[20][1], 0.0))

sim.add_player('Acura_NSX_red.png', 4.4, unicycle_model, init_pose[0])
sim.add_player('Acura_NSX_yellow.png', 4.4, unicycle_model, init_pose[1])
sim.add_player('Suzuki_DR200S.png', 3.0, unicycle_model, init_pose[2])

while not sim.done:
    sim.update_player(0, (2,0.1))
    sim.update_player(1, (2,-0.1))
    sim.update_player(2, (3,0.2))
    sim.tick()
