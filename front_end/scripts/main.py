import rospy

class frontend():
    def __init__(self) -> None:

        self.publisher_ = rospy.Publisher('TODO', TODO, queue_size=10)
        self.subscriber_ = rospy.Subscriber('TODO', TODO, self.main_callback)

    def frontend_callback(self, msg: TODO):
        pass

def main():
    rospy.init_node("Front-End")

    frontend = frontend()

    rospy.spin()


if __name__ == '__main__':
    main()