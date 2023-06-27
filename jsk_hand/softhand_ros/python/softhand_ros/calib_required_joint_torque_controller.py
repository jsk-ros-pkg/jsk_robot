from dynamixel_controllers.joint_torque_controller \
    import JointTorqueController

from softhand_ros.calib_required_controller import CalibRequiredController


class CalibRequiredJointTorqueController(
        JointTorqueController, CalibRequiredController
):
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        JointTorqueController.__init__(
            self, dxl_io, controller_namespace, port_namespace)
        CalibRequiredController.__init__(self)

    def initialize(self):
        if not JointTorqueController.initialize(self):
            return False
        return CalibRequiredController.calib_initialize(self)
