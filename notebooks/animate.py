#!/usr/bin/env python3
"""Script for creating an animation from a JUST VFH agent log file."""

import argparse

from matplotlib import animation
import matplotlib.pyplot as plt
import numpy as np
import h5py

# pylint: disable=no-member

def main():
    """The main fn."""
    parser = argparse.ArgumentParser(
        description='Create an animation of the window & polar histograms from a JUST log file',
    )
    parser.add_argument('-a', '--agent-name', default='jerry')
    parser.add_argument('--fps', default=100, type=int)

    args = parser.parse_args()

    with h5py.File(f'/tmp/just/{args.agent_name}/log.h5', 'r') as file:  # TODO: parameterize

        dset1 = file['vfh_agent/polar_histogram']
        dset2 = np.array(file['vfh_agent/window_histogram'])
        theta = np.linspace(0.0, 2 * np.pi, dset1.shape[0], endpoint=False)

        window_size = 30

        fig = plt.figure()
        axes = [plt.subplot(1, 2, 1, projection='polar'), plt.subplot(1, 2, 2)]

        bars = axes[0].bar(theta, dset1[:,0])
        axes[0].set_ylim(0,dset1[:,:].max())
        window = axes[1].imshow(
            dset2[:,0].reshape(window_size,window_size),
            interpolation='none',
            origin='lower',
        )

        def init():
            return bars, window

        def animate(i):
            for j, bar in enumerate(bars):      # pylint: disable=disallowed-name
                bar.set_height(dset1[j,i])
            data = dset2[:,i].reshape(window_size, window_size)
            data[window_size // 2 - 1, window_size // 2 - 1] = 50
            window.set_data(data)
            return bars, window

        interval = 1000 // args.fps
        anim = animation.FuncAnimation(fig, animate, init_func=init,
                                       frames=dset1.shape[1], interval=interval, blit=False)
        anim.save('just_animation.mp4', writer=animation.FFMpegWriter(fps=args.fps))

if __name__ == '__main__':
    main()
