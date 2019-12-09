import numpy as np

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

def get_sweeping(vector, index=-1, interval=-1):
    index_provided = index >= 0
    interval_provided = interval >= 0

    sweep = []
    if not index_provided and not interval_provided:
        sweep = [i for i in range(len(vector))]
    elif index_provided:
        # How many points does the sweep contain? Interval is on the left and on the right of the index
        if interval_provided:
            npoints = interval*2 + 1
        else:
            npoints = len(vector)

        current_index = index
        current_step = 1
        current_sign = -1
        sweep.append(current_index)
        for i in range(npoints-1):
            current_index += current_step*current_sign
            current_step += 1
            current_sign = -current_sign
            
            actual_index = current_index
            if current_index < 0:
                actual_index = len(vector) + current_index
            elif current_index >= len(vector):
                actual_index = current_index - len(vector)

            sweep.append(actual_index)

    return sweep
        
# WARNING this just checks if two triangles have two points in common. 
def is_adjacent(t1, t2):
    found1 = [(t1[i] in t2) for i in range(3)]

    eq_points = int(found1[0]) + int(found1[1]) + int(found1[2])
    if eq_points < 2:
        return False
    else:
        return True
        # found2 = [(t2[i] in t1) for i in range(3)]
        # for i in range(3):
        #     if found1[i] == 0:
        #         ext_index1 = i
        # for i in range(3):
        #     if found2[i] == 0:
        #         ext_index2 = i    