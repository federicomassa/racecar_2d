from sim2d import Sim2D
import numpy

def unicycle_model(current_pose, controls, time_step, *argv):
    if len(current_pose) != 3:
        raise Exception("current_pose must be of length 3: x,y,theta")
    if len(controls) != 2:
        raise Exception("controls must be of length 2: v, omega")

    new_pose = current_pose
    new_pose[0] += controls[0]*numpy.cos(current_pose[2])*time_step
    new_pose[1] += controls[0]*numpy.sin(current_pose[2])*time_step
    new_pose[2] += controls[1]*time_step

    return new_pose

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
