import numpy as np
import matplotlib.pyplot as plt

# Trajectory to follow
x = np.linspace(0.0, 10.0, 11)
y = np.sin(x)
v = (x+2.1)*0.5

trajectory = [(x[i],y[i],v[i]) for i in range(len(x))]

t_max = 10.0
t_step = 0.01

xnew = []
ynew = []
thetanew = []
vnew = []

for i in range(len(trajectory)-1):
    delta_s = np.sqrt(np.square(y[i+1]-y[i]) + np.square(x[i+1]-x[i]))
    a = (np.square(v[i+1]) - np.square(v[i]))/(2*delta_s)
    delta_t = (v[i+1]-v[i])/a
    N_steps = int(np.ceil(delta_t/t_step))
    if N_steps == 0:
        raise Exception('t_step too large')

    jmin = len(xnew)
    jmax = len(xnew) + N_steps
    for j in range(jmin,jmax):
        if j == jmin:
            print('appending ', len(xnew), x[i])
            xnew.append(x[i])
            ynew.append(y[i])
            thetanew.append(np.arctan2(y[i+1]-y[i],x[i+1]-x[i]))
            vnew.append(v[i]) 
        else:
            xnew.append(xnew[j-1] + vnew[j-1]*np.cos(thetanew[j-1])*t_step)
            ynew.append(ynew[j-1] + vnew[j-1]*np.sin(thetanew[j-1])*t_step)
            thetanew.append(thetanew[j-1])
            vnew.append(vnew[j-1] + a*t_step)


plt.plot(x,y,'o')
plt.plot(xnew,ynew,'-x')
plt.show()
    




