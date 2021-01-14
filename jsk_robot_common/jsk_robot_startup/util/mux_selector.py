#!/usr/bin/env python


"""
mux_selector.py : check and select mux input topic on condition of specified topic

Usage:

rosrun jsk_robot_startup mux_selector.py /joy1 'm.buttons[9]==1' /cmd_vel1 /joy2 'm.buttons[9]==1' /cmd_vel2
This node takes three arguments for one topic.
The first one is the topic to be monitored.
When a message from this topic is received, it is assigned as a variable `m`.
If a condition specified as the second argument,
this node calls a service to select the topic specified as the third argument.

If the type of the monitored topic is unknown, you can set the type with `~topics` rosparam.
`~topics` should be the list like
[{'name': '/joy1', 'type': 'sensor_msgs/Joy'}, {'name': '/joy2', 'type': 'sensor_msgs/Joy'}, ...]
"""

import rospy
import thread
import sys
import roslib.message
import rostopic
from topic_tools.srv import MuxSelect
import traceback


def callback (m, expr, topic, index):
    global selects, lockobj
    lockobj.acquire()
    try:
        ok = eval(expr)
    except Exception as e:
        rospy.logerr("Failed to check condition (%s): %s" % (expr, str(e)))
        lockobj.acquire()
        return
    try:
        if(ok):
            selects[index] = (topic, rospy.Time.now())
        else:
            selects[index] = (None, rospy.Time.now())
    except Exception as e:
        rospy.logerr('Error: '+str(e))
        rospy.logerr(traceback.format_exc())
    lockobj.release()


def gen_callback(expr, select, index):
    return (lambda m: callback(m, expr, select, index))


def add_trigger(topic, expr, select, index, wait=False):
    global topics
    if topic in topics:
        topic_type = topics[topic]
        topic_class = roslib.message.get_message_class(topic_type)
    else:
        topic_type, _, _ = rostopic.get_topic_type(topic)
        topic_class, _, _ = rostopic.get_topic_class(topic)

    if topic_type is None:
        if wait is False:
            rospy.loginfo('%s is not published yet', topic)
            return None
        elif wait is True:
            rate = rospy.Rate(1)
            while not rospy.is_shutdown() and topic_type is None:
                topic_type, _, _ = rostopic.get_topic_type(topic)
                topic_class, _, _ = rostopic.get_topic_class(topic)
                rospy.loginfo('waiting topic %s' % topic)
                rate.sleep()
        else:
            raise ValueError('wait should be bool')

    if(topic_class is None):
        rospy.loginfo('%s is not builded yet', topic_type)
        return None

    try:
        cb_obj = gen_callback(expr, select, index)
        rospy.loginfo("start subscribe (topic=%s type=%s)", topic, topic_type);
        sub = rospy.Subscriber(topic,topic_class,cb_obj)
    except Exception, e:
        rospy.loginfo(str(e))
        return None

    return sub


def update_trigger(conditions, wait=False):
    # setting triggers
    global subs
    for index in range(len(conditions)):
        if subs[index] is not None:
            continue
        cond = conditions[index]
        subs[index] = add_trigger(cond[0],cond[1],cond[2],index,
                                  wait=wait)


if __name__ == "__main__":
    global selects, subs, topics
    global lockobj
    lockobj = thread.allocate_lock()
    selects = []
    subs = []
    topics = {}

    # parse arguments
    conditions = [x for x in sys.argv[1:] if not ':=' in x]

    if len(conditions) % 3 != 0:
        rospy.logerr('args must be (read_topic condition select_topic)...')
        exit(0)

    conditions = [(conditions[i],conditions[i+1],conditions[i+2])
                  for i in range(len(conditions)) if i % 3 == 0]

    # ros node initialization
    rospy.init_node('mux_selector')
    deadtime = rospy.get_param('~patient', 0.5)
    freq = rospy.get_param('~frequency', 20.0)
    default_select = rospy.get_param('~default_select', None)
    wait = rospy.get_param('~wait', False)

    mux_name_ = rospy.resolve_name('mux')
    rospy.wait_for_service(mux_name_+'/select')
    mux_client = rospy.ServiceProxy(mux_name_+'/select',MuxSelect)

    size = len(conditions)
    selects = [(None, rospy.Time(0))] * size
    subs = [None] * size

    topic_list = rospy.get_param('~topics', [])
    for topic in topic_list:
        topics[topic['name']] = topic['type']

    # loop
    try:
        before = default_select
        update_trigger(conditions, wait=wait)
        looprate = rospy.Rate(freq)
        while not rospy.is_shutdown():
            lockobj.acquire()
            cand = [x[0] for x in selects if x[0] is not None and (rospy.Time.now()-x[1]).to_sec()<deadtime]
            lockobj.release()

            if cand != []:
                next_topic = cand[0]
            else:
                next_topic = default_select
            try:
                if not (before == next_topic or next_topic is None):
                    mux_client(next_topic)
            except rospy.ServiceException, e:
                rospy.loginfo("Service did not process request: %s", str(e))
            before = next_topic
            looprate.sleep()
    except Exception, e:
        rospy.logerr('Error: '+str(e))
        rospy.logerr(traceback.format_exc())
