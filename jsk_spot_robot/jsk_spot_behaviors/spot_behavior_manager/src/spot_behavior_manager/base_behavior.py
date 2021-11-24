import rospy
import roslaunch


def load_behavior_class(class_string):
    package_name = class_string.split('.')[0]
    module_name = class_string.split('.')[1]
    class_name = class_string.split('.')[2]
    behavior_class = getattr(getattr(__import__(package_name),module_name),class_name)
    return behavior_class


class BaseBehavior(object):

    def __init__(self, spot_client, sound_client):

        self.spot_client = spot_client
        self.sound_client = sound_client

    def run_initial(self, start_node, end_node, edge, pre_edge ):
        pass

    def run_main(self, start_node, end_node, edge, pre_edge ):
        pass

    def run_final(self, start_node, end_node, edge, pre_edge ):
        pass


class SimpleBehavior(BaseBehavior):

    def run_initial(self, start_node, end_node, edge, pre_edge ):

        rospy.loginfo('__run_initial() called')

    def run_main(self, start_node, end_node, edge, pre_edge ):

        rospy.loginfo('__run_main() called')
        rospy.loginfo('start_node: {}'.format(start_node))
        rospy.loginfo('end_node: {}'.format(end_node))
        rospy.loginfo('edge: {}'.format(edge))
        rospy.loginfo('pre_edge: {}'.format(pre_edge))
        return True

    def run_final(self, start_node, end_node, edge, pre_edge ):

        rospy.loginfo('__run_finalize() called')
