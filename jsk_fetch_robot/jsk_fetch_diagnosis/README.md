# jsk_fetch_diagnosis

publish the result of read_board command for debugging of fetch motor drivers

## Usage

```bash
$ catkin build jsk_fetch_diagnosis
$ source <path to catkin workspace>/devel/setup.bash

$ rosrun jsk_fetch_diagnosis check_driver_boards.py --help
usage: check_driver_boards.py [-h] [--topicname TOPICNAME]

optional arguments:
  -h, --help            show this help message and exit
    --topicname TOPICNAME
                            topicname of result of read_board commands

$ rosrun jsk_fetch_diagnosis check_driver_boards.py
// then the node publish the results of read_board command to /check_board_info topic

// Open another terminal
$ source <path to catkin workspace>/devel/setup.bash
$ rostopic echo /check_board_info
```
