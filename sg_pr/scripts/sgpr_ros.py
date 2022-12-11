#!/home/hao/anaconda3/envs/sgpr/bin/python
import os
import sys
import json
import rospy
import random
import numpy as np
import matplotlib.pyplot as plt

# from std_msgs.msg import Float32MultiArray

from sg_pr.msg import EvalPackage, EvalScore

from SG_PR.utils import tab_printer
from SG_PR.sg_net import SGTrainer
from SG_PR.parser_sg import sgpr_args


class SGPR_ros:
    def __init__(self, trainer) -> None:
        self.trainer = trainer

        self.publisher_ = rospy.Publisher('score', EvalScore, queue_size=10)
        self.subscriber_ = rospy.Subscriber('sg', EvalPackage, self.sgpr_callback)
        colors = ["#"+''.join([random.choice('0123456789ABCDEF') for j in range(6)])
          for i in range(10)]
        a = np.arange(10)
        self.colors = dict(zip(a, colors))

    def sgpr_callback(self, msg: EvalPackage):
        print("SG_PR Callback...............")
        target = {"centers": [[p.x, p.y, p.z]for p in msg.target.centers], 
                  "nodes": msg.target.nodes, 
                  "pose": [0 for _ in range(12)]}
        batch = [{"centers": [[p.x, p.y, p.z]for p in graph.centers], 
                  "nodes": graph.nodes, 
                  "pose": [0 for _ in range(12)]} for graph in msg.batch]
        print("target tags: ", msg.target.nodes)
        print("batch tags: ", [graph.nodes for graph in msg.batch]) 

        # with open(f'./target_{msg.receipt}.json', 'w') as fw:
        #     json.dump(target, fw)
        # with open(f'./batch__{msg.receipt}.json', 'w') as fw:
        #     json.dump(batch, fw)
        pred, gt = self.trainer.eval_package(target, batch) #TODO: batch size 128

        msg = EvalScore(receipt = msg.receipt)
        msg.score = pred
        fig = plt.figure()
        # plt.ion()
        ax = fig.add_subplot(projection='3d')
        for i in range(len(target["centers"])):
            xs = target["centers"][i][0]
            ys = target["centers"][i][1]
            zs = target["centers"][i][2]
            n = target["nodes"][i]
            ax.scatter(xs, ys, zs, color=self.colors[n])
        plt.savefig(f'./results/{msg.receipt}.png')
        self.publisher_.publish(msg)


def main():
    rospy.init_node("sgpr")

    args = sgpr_args()
    args.load('./config/config.yml')
    args.load(os.path.abspath('./config/config.yml'))
    tab_printer(args)
    trainer = SGTrainer(args, False)
    trainer.model.eval()

    sgpr = SGPR_ros(trainer)
    
    rospy.spin()


if __name__ == '__main__':
    main()