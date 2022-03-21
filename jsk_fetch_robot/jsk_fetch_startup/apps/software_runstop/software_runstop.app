display: Fetch software runstop
platform: fetch
run: rostopic/rostopic
run_args: "pub -1 /enable_software_runstop std_msgs/Bool 'data: true'"
interface: jsk_fetch_startup/software_runstop.interface
icon: jsk_fetch_startup/software_runstop.png
