#!/usr/bin/env python

import rospy
from fcu_common.msg import FW_State
from rosgraph_msgs.msg import Clock

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.lines import Line2D

class state_subscriber(): # will only extract x,y data for plotting

    def __init__(self):
        #----------------------States-------------------------------
        self.pn = 0.
        self.pe = 0.
        #------------------------------------------------------------
        rospy.Subscriber("/junker/truth", FW_State, self.callback)
        self.rate = 100 # 100 hz

    def callback(self, FW_State):
        self.pn = FW_State.position[0]
        self.pe = FW_State.position[1]

    def print_states(self):
        print "pn: ", self.pn
        print "pe: ", self.pe

rospy.init_node('plotter', anonymous=True)
states = state_subscriber()

pn_data	 = np.array([]) # NORTHern position
pe_data	 = np.array([]) # EASTern position

pn_max	= 7.0 # DO I NEED ALL OF THIS?+++++++++++++++++++++++++++
pn_min	=-7.0
pe_max	= 7.0
pe_min	=-7.0
#------------------------------------------------------------
# set up figure and animation for plotting of states
fig = plt.figure()
ax = fig.add_subplot(111)

#ax.add_line(line)
plot_line, = ax.plot([], [])
ax.set_xlim(pe_min, pe_max)
ax.set_ylim(pn_min, pn_max)

def init_plot1():
    """initialize animation"""
    plot_line.set_data([], [])
    return plot_line

def animate_plot1(i): # Perform animation step

    global states, pn_data, pe_data
    global ax, fig
    global pn_max, pn_min, pe_max, pe_min
    need_to_plot = False
    # The append function doesn't append to the array given by reference, so we
    # have to pass it by value and simultaneously assign it to the original.
    pn_data	 = np.append(pn_data, states.pn)
    pe_data	 = np.append(pe_data, states.pe)
    # update the x-axis when necessary...
    if(pe_min > pe_data.min()):
        pe_min = pe_data.min() - 1.0
        ax.set_xlim(pe_min, pe_max)
        need_to_plot = True

    if(pe_max < pe_data.max()):
        pe_max = pe_data.max() + 1.0
        ax.set_xlim(pe_min, pe_max)
        need_to_plot = True

    # update the y-axis when necessary...
    if(pn_min > pn_data.min()):
        pn_min = pn_data.min() - 1.0
        ax_pn.set_ylim(pn_min, pn_max)
        need_to_plot = True

    if(pn_max < pn_data.max()):
        pn_max = pn_data.max() + 1.0
        ax_pn.set_ylim(pn_min, pn_max)
        need_to_plot = True

    # update the plot if any of the axis limits have changed
    if need_to_plot:
        fig.show()

    plot_line.set_data(pe_data,	pn_data)
    return plot_line

# choose the interval based on dt and the time to animate one step
from time import time
t0 = time()
animate_plot1(0)
#animate_plot2(0)
t1 = time()
interval = 1000 * (1./30.) - (t1 - t0)

ani1_plot = animation.FuncAnimation(fig, animate_plot1, frames=300,
								interval=interval, blit=True, init_func=init_plot1)

plt.show()
