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
sim.set_track('track.json')
sim.set_model(unicycle_model)

while not sim.done:
    sim.update((3, 0.16))
    sim.sleep()
