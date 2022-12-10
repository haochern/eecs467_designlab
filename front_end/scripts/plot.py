#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from utils import *

import rospy
import ros_numpy


class Plot:
    
    def __init__(self) -> None:
        self.sub = rospy.Subscribe('sg', self.plot_callback, queue_size=10)
        colors = ["#"+''.join([random.choice('0123456789ABCDEF') for j in range(6)])
          for i in range(10)]
        a = np.arange(10)
        self.colors = dict(zip(a, colors))

    def plot_callback(self, msg):
        target = {"centers": [[p.x, p.y, p.z]for p in msg.target.centers], 
                  "nodes": msg.target.nodes, 
                  "pose": [0 for _ in range(12)]}
        batch = [{"centers": [[p.x, p.y, p.z]for p in graph.centers], 
                  "nodes": graph.nodes, 
                  "pose": [0 for _ in range(12)]} for graph in msg.batch]

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        for i in range(len(target["centers"])):
            xs = target["centers"][i][0]
            ys = target["centers"][i][1]
            zs = target["centers"][i][2]
            n = target["nodes"][i]
            ax.scatter(xs, ys, zs, color=self.colors[n])
        plt.show()





def main():
    rospy.init_node("plot")

    f = Plot()

    rospy.spin()


if __name__ == '__main__':
    main()