#!/home/hao/anaconda3/envs/sgpr/bin/python
import os

import rospy

from std_msgs.msg import Float32MultiArray

from sg_pr.msg import EvalPackage

from SG_PR.utils import tab_printer
from SG_PR.sg_net import SGTrainer
from SG_PR.parser_sg import sgpr_args


class sgpr_ros():
    def __init__(self, trainer) -> None:
        self.trainer = trainer

        self.publisher_ = rospy.Publisher('score', Float32MultiArray, queue_size=10)
        self.subscriber_ = rospy.Subscriber('eval_package', EvalPackage, self.sgpr_callback)

    def sgpr_callback(self, msg: EvalPackage):
        target = {"centers": msg.target.centers, 
                  "nodes": msg.target.nodes, 
                  "pose": [0 for _ in range(12)]}
        batch = [{"centers": graph.centers, 
                  "nodes": graph.nodes, 
                  "pose": [0 for _ in range(12)]} for graph in msg.batch.array]

        pred, gt = self.trainer.eval_package(target, batch) #TODO: batch size 128

        score = Float32MultiArray()
        score.data = pred
        self.publisher_.publish(score)


def main():
    rospy.init_node("sgpr")

    args = sgpr_args()
    args.load('./config/config.yml')
    args.load(os.path.abspath('./config/config.yml'))
    tab_printer(args)
    trainer = SGTrainer(args, False)
    trainer.model.eval()

    sgpr = sgpr_ros(trainer)
    
    rospy.spin()


if __name__ == '__main__':
    main()