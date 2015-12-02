Samples
=======

- chatter_sample

This will publish to each other the "Hello Wolrd".

```
In One Terminal
$ rossetpr1040
$ roslaunch jsk_pr2_startup chatter_pr1040_rocon.launch

In Another Terminal
$ rossetpr1012
$ roslaunch jsk_pr2_startup chatter_pr1012_rocon.launch
```

- look_sample

PR1040 will look at PR1012.

```
In One Terminal
$ rossetpr1040
$ roslaunch jsk_pr2_startup look_pr1040_rocon.launch

In Another Terminal
$ rossetpr1012
$ roslaunch jsk_pr2_startup look_pr1012_rocon.launch
```
- cameraman_sample

PR1040 will become cameraman , get PR1012 pos and direct the grapsed camera to PR1012.

```
In One Terminal
$ rossetpr1040
$ roslaunch jsk_pr2_startup cameraman_pr1040_rocon.launch

In Another Terminal
$ rossetpr1012
$ roslaunch jsk_pr2_startup cameraman_pr1012_rocon.launch
```