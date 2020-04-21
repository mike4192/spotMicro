#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from ..fof import FirstOrderFilter



fig, ax = plt.subplots(1,1)

dt = 0.05
fof = FirstOrderFilter(dt=0.05, tau=1, x0=0)

# Use arrange to create an array defined by a start, end, and step size
# np.arrang(start_num, end_num + step_size, step_size)
time = np.arange(0, 15+dt, dt)

out = []

fof.set_command(1)

for t in time:

    # Halfway through, set a new command
    if t == 7.5:
        fof.set_command(0)

    y = fof.run_timestep_and_get_output()
    out.append(y)
    

ax.plot(time,np.array(out))
ax.grid(True)
ax.set_title('First Order Response to Step Change at 0 and 7.5 sec \n Time Constant = 1 sec, dt = 0.05 sec')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Output')
plt.show()
