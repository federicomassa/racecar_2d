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
                ins.append((n,m))

        for j, _ in enumerate(o):
            if j%take_every == 0:
                c = o[j]
                x = c[0]
                y = c[1]
                out.append((x,y))

        for e, _ in enumerate(p):
            if e%take_every == 0:
                z = p[e]
                g = z[0]
                h = z[1]
                race.append((g,h))

    return race,ins,out