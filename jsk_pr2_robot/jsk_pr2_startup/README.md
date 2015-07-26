# jsk_pr2_startup

## setup

## specify robot identifier

Write to `~/.bashrc`

```bash
export ROBOT_NAME=pr1012  # or pr1040, etc.
```

###. rewrite `/etc/ros/robot.launch`

Please rewrite `/etc/ros/robot.launch` like following:
```xml
<launch>

    <!-- Robot Description --> <param name="robot_description" textfile="/etc/ros/groovy/urdf/robot.xml" />

    <!-- Robot Analyzer --> <rosparam command="load" file="$(find pr2_bringup)/config/pr2_analyzers.yaml" ns="diag_agg" />

    <!-- Robot bringup --> 
    <include file="$(find jsk_pr2_startup)/pr2_bringup.launch" />
    <!-- <group> -->
    <!--   <remap from="/joy" to="/joy_org"/> -->
    <!--   <include file="$(find pr2_bringup)/pr2.launch" /> -->
    <!-- </group> -->

    <!-- Web ui --> <!-- include file="$(find webui)/webui.launch" /> -->

    <!-- Android app --> <include file="$(find local_app_manager)/app_manager.launch" >
      <arg name="ROBOT_NAME" value="pr1012" />
      <arg name="ROBOT_TYPE" value="pr2" />
    </include>

    <!-- RobotWebTools --> <include file="$(find rwt_image_view)/launch/rwt_image_view.launch"/>

    <!-- kinect -->
    <include file="$(find jsk_pr2_startup)/jsk_pr2_sensors/kinect_head.launch">
      <arg name="respawn" value="false" />
    </include>
    <rosparam file="/etc/ros/robot.yaml"/>
</launch> 

```

### launch mongodb for multiple users

Different users in same unix group can't run mongod against single db owned by that group.
This is because `mongod` opens database files using the `O_NOATIME` flag to the open system call.
Open with `O_NOATIME` only works if the UID completelly matchs or the caller is priviledged (`CAP_FOWNER`) for security reasons.
So if you want to launch mongodb with shared database resouces, it's better to use POSIX Capabilities in Linux.

```bash
# In Ubuntu
$ sudo aptitude install libcap2-bin
$ sudo setcap cap_fowner+ep /usr/bin/mongod
```

