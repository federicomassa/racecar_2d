import sys
from math import sqrt
import json

# Parse json file with Inside track, Outside track, Racing track
# Take one point of the json every <take_every>
# Returns a list of points of race, inside and outside
def json_parser(json_file, take_every=1):

    race = []
    ins = []
    out = []

    tol = 0.001

    with open(json_file) as data_file:
        data = json.loads(data_file.read())
        o = data['Outside']
        t = data['Inside']
        p = data['Racing']
        for i, _ in enumerate(t):
            if i%take_every == 0:
                d = t[i]
                n = d[0]
                m = d[1]

                # Check duplicates
                if len(ins) == 0:
                    ins.append((n,m))
                elif abs(n - ins[-1][0]) > tol or abs(m - ins[-1][1]) > tol:
                    ins.append((n,m))

        # Check duplicates between first and last point
        if abs(ins[0][0] - ins[-1][0]) < tol and abs(ins[0][1] - ins[-1][1]) < tol :
            ins = ins[0:-1]

        for j, _ in enumerate(o):
            if j%take_every == 0:
                c = o[j]
                x = c[0]
                y = c[1]

                # Check duplicates
                if len(out) == 0:
                    out.append((x,y))
                elif abs(x - out[-1][0]) > tol or abs(y - out[-1][1]) > tol:
                    out.append((x,y))

        # Check duplicates between first and last point
        if abs(out[0][0] - out[-1][0]) < tol and abs(out[0][1] - out[-1][1]) < tol :
            out = out[0:-1]

        for e, _ in enumerate(p):
            if e%take_every == 0:
                z = p[e]
                g = z[0]
                h = z[1]

                # Check duplicates
                if len(race) == 0:
                    race.append((g,h))
                elif abs(g - race[-1][0]) > tol or abs(h - race[-1][1]) > tol:
                    race.append((g,h))


        # Check duplicates between first and last point
        if abs(race[0][0] - race[-1][0]) < tol and abs(race[0][1] - race[-1][1]) < tol :
            race = race[0:-1]

    return race,ins,out