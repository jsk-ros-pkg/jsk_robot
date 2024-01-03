display: Interrupt
platform: all
run: rostopic/rostopic
run_args: "pub -1 /roseus_resume/interrupt std_msgs/Empty"
interface: jsk_robot_startup/roseus_resume_interrupt.interface
icon: jsk_robot_startup/roseus_resume_interrupt.png
