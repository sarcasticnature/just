#!/bin/env python3
from matplotlib import animation
import matplotlib.pyplot as plt
import numpy as np
import h5py

f = h5py.File('/tmp/just/jerry/foo.h5', 'r')

dset1 = f['vfh_agent/polar_histogram']
dset2 = np.array(f['vfh_agent/window_histogram'])
theta = np.linspace(0.0, 2 * np.pi, dset1.shape[0], endpoint=False)

window_size = 30

fig = plt.figure()
axes = [plt.subplot(1, 2, 1, projection='polar'), plt.subplot(1, 2, 2)]

bars = axes[0].bar(theta, dset1[:,0])
axes[0].set_ylim(0,dset1[:,:].max())
window = axes[1].imshow(dset2[:,0].reshape(window_size,window_size), interpolation='none', origin='lower')

def init():
    return bars, window

def animate(i):
    for j, b in enumerate(bars):
        b.set_height(dset1[j,i])
    #axes[0].set_ylim(0,dset1[:,i].max())
    foo = dset2[:,i].reshape(window_size, window_size)
    foo[window_size // 2 - 1, window_size // 2 - 1] = 50
    window.set_data(foo)
    return bars, window

anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=dset1.shape[1], interval=10, blit=False)
anim.save('basic_animation.mp4', writer=animation.FFMpegWriter(fps=100))
