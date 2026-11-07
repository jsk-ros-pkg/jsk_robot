"""jsk_ros1_ros2_compat.rospy_rclpy_compat with local fallbacks (older releases, ROS 1 without jsk_common 3)."""

from __future__ import annotations

import os
import sys

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as _upstream
except ImportError:
    if int(os.environ.get("ROS_VERSION", "1")) == 2:
        raise
    _upstream = None

ROS_VERSION = int(os.environ.get("ROS_VERSION", "1"))

if ROS_VERSION == 2:
    import rclpy
    import rclpy.executors
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    def _qos(depth: int, latch: bool):
        if not latch:
            return depth
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

else:
    import rospy


def _upstream_has(name: str, *params: str) -> bool:
    func = getattr(_upstream, name, None)
    if func is None:
        return False
    if not params:
        return True
    import inspect

    return all(param in inspect.signature(func).parameters for param in params)


class ros_compat:  # noqa: N801 - used like the upstream module
    @staticmethod
    def init_node(name: str, anonymous: bool = False):
        if _upstream_has("init_node"):
            return _upstream.init_node(name, anonymous=anonymous)
        if ROS_VERSION == 2:
            if not rclpy.ok():
                rclpy.init(args=sys.argv)
            return rclpy.create_node(name)
        rospy.init_node(name, anonymous=anonymous)
        return None

    @staticmethod
    def spin(node, multi_threaded: bool = False) -> None:
        """rospy.spin() or spin until shutdown (multi_threaded: for callbacks calling services)."""
        if ROS_VERSION == 1:
            rospy.spin()
            return
        from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

        executor = MultiThreadedExecutor() if multi_threaded else SingleThreadedExecutor()
        if _upstream_has("spin", "executor"):
            return _upstream.spin(node, executor)
        try:
            executor.add_node(node)
            executor.spin()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()

    @staticmethod
    def create_publisher(node, topic, msg_type, queue_size=1, latch=False):
        if _upstream_has("create_publisher", "latch"):
            return _upstream.create_publisher(node, topic, msg_type, queue_size=queue_size, latch=latch)
        if ROS_VERSION == 2:
            return node.create_publisher(msg_type, topic, _qos(queue_size, latch))
        return rospy.Publisher(topic, msg_type, queue_size=queue_size, latch=latch)

    @staticmethod
    def create_subscription(node, topic, msg_type, callback, queue_size=1, latch=False):
        if _upstream_has("create_subscription", "latch"):
            return _upstream.create_subscription(
                node, topic, msg_type, callback, queue_size=queue_size, latch=latch
            )
        if ROS_VERSION == 2:
            return node.create_subscription(msg_type, topic, callback, _qos(queue_size, latch))
        return rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size)

    @staticmethod
    def set_param(node, name, value) -> None:
        if _upstream_has("set_param"):
            return _upstream.set_param(node, name, value)
        if ROS_VERSION == 2:
            from rclpy.parameter import Parameter

            if not node.has_parameter(name):
                node.declare_parameter(name, value)
            node.set_parameters([Parameter(name, value=value)])
        else:
            rospy.set_param(name, value)

    @staticmethod
    def loginfo(node, msg):
        if _upstream is not None:
            return _upstream.loginfo(node, msg)
        return rospy.loginfo(msg)

    @staticmethod
    def logwarn(node, msg):
        if _upstream is not None:
            return _upstream.logwarn(node, msg)
        return rospy.logwarn(msg)

    @staticmethod
    def logerr(node, msg):
        if _upstream is not None:
            return _upstream.logerr(node, msg)
        return rospy.logerr(msg)

    @staticmethod
    def stamp_now(node):
        if _upstream is not None:
            return _upstream.stamp_now(node)
        return rospy.Time.now()

    @staticmethod
    def create_timer(node, period_sec, callback):
        """``callback(event)``; event is None on ROS 2."""
        if _upstream is not None:
            return _upstream.create_timer(node, period_sec, callback)
        return rospy.Timer(rospy.Duration(period_sec), callback)

    @staticmethod
    def create_service(node, name, srv_type, callback):
        """``callback(request, response)`` returns the response on both."""
        if _upstream is not None:
            return _upstream.create_service(node, name, srv_type, callback)

        def _wrapped(request):
            return callback(request, srv_type._response_class())

        return rospy.Service(name, srv_type, _wrapped)


def node_argv(argv: list[str] | None = None) -> list[str]:
    """Command line arguments without ROS 1 remappings / ROS 2 --ros-args."""

    argv = list(sys.argv if argv is None else argv)
    if "--ros-args" in argv:
        argv = argv[: argv.index("--ros-args")]
    return [arg for arg in argv[1:] if ":=" not in arg]


__all__ = ["ROS_VERSION", "node_argv", "ros_compat"]
