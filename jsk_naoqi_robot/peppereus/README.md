peppereus
=========

This is a NAO-specific package for roseus interface.

How to make pepper model on euslisp
-----------------------------------

Install pepper mesh files with manual approval of license
```
sudo apt-get install ros-<ros version>-pepper-meshes
catkin build
```

Control Pepper via roseus
-------------------------

```
(load "package://peppereus/pepper-interface.l") ;; load modules
(setq *pepper* (pepper))          ;; creat a robot model
(setq *ri* (instance pepper-interface :init)) ;; make connection to the real robot
(objects (list *pepper*))        ;; display the robot model
```
or

```
(load "package://peppereus/pepper-interface.l") ;; load modules
(pepper-init)
```

How to try methods
------------------

1. [roslaunch jsk_pepper_startup.launch](../jsk_pepper_statup/README.md)
2. Please refer to `Control Pepper via roseus`.
3. Please try methods, you can refer to the explanations below how to try them. If there is a sign of `kochigami-develop`, please follow [Interface when controlling NAO and Pepper via roseus](../README.md). 

Methods
-------

***Tablet***  

By using tablet-related methods, we can show image, app, html file and web page on the tablet.  

- `:show-image file` (naoqi_bridge [kochigami-develop]/ tablet.launch) 

Show an image on the tablet, using the cache. The image should be under `/home/nao/.local/share/PackageManager/apps/<Folder path>/html/` inside a robot.  
(Default is "/home/nao/.local/share/PackageManager/apps/img/html/")  You can change 'Folder path' by using `:set-show-image-folder-path name`.

`file`: file name (str)

[ALTabletService::showImage](http://doc.aldebaran.com/2-5/naoqi/core/altabletservice-api.html#ALTabletService::showImage__ssCR)  
[related PR](https://github.com/ros-naoqi/naoqi_bridge/pull/52)

```
; put 'test.jpg' under /home/nao/.local/share/PackageManager/apps/img/html/
send *ri* :show-image "test.jpg"

; The ip of the robot from the tablet is 198.18.0.1, and this parent service actually calls ALTabletService::showImage("http://198.18.0.1/img/test.jpg").
```

- `:set-show-image-folder-path name` (naoqi_bridge [kochigami-develop]/ tablet.launch)

Change the path of a file which you want to show on the tablet. This method changes the part of `Folder path` in `/home/nao/.local/share/PackageManager/apps/<Folder path>/html/`.  

`name`: directory name (str, default is `img`)  

[related PR](https://github.com/ros-naoqi/naoqi_bridge/pull/52)

```
send *ri* :set-show-image-folder-path "aaa/bbb"
#<naoqi_bridge_msgs::setfolderpathresponse #X9b1c660>

; put 'test.jpg' under /home/nao/.local/share/PackageManager/apps/aaa/bbb/html/
; (send *ri* :get-show-image-folder-path) will return "/home/nao/.local/share/PackageManager/apps/aaa/bbb/html/"

send *ri* :show-image "test.jpg"
```

- `:get-show-image-folder-path` (naoqi_bridge [kochigami-develop]/ tablet.launch)

Get the current path of a file which you want to show on the tablet.  

[related PR](https://github.com/ros-naoqi/naoqi_bridge/pull/52)

```
send *ri* :get-show-image-folder-path
"/home/nao/.local/share/PackageManager/apps/img/html/"
```

- `:show-app app` (naoqi_bridge [kochigami-develop]/ tablet.launch)

Start new application on tablet and shows it. The index.html file of the app should be in /home/nao/.local/share/PackageManager/apps/<app>/html/. 'app' is a parameter of this method.

`app`: app name (str)

[ALTabletService::loadApplication](http://doc.aldebaran.com/2-5/naoqi/core/altabletservice-api.html#ALTabletService::loadApplication__ssCR)  
[ALTabletService::showWebview](http://doc.aldebaran.com/2-5/naoqi/core/altabletservice-api.html#altabletservice-showwebview1)  
[related PR](https://github.com/ros-naoqi/naoqi_bridge/pull/52)

```
; create app named 'img' under /home/nao/.local/share/PackageManager/apps/img/html/
send *ri* :show-app "img"
```

- `:show-webview url` (naoqi_bridge [kochigami-develop]/ tablet.launch)

Display the webview on the tablet and load the url.

`url`: url of the web page (str)

[ALTabletService::showWebview](http://doc.aldebaran.com/2-5/naoqi/core/altabletservice-api.html#ALTabletService::showWebview__ssCR)  
[related PR](https://github.com/ros-naoqi/naoqi_bridge/pull/52)

```
send *ri* :show-webview "http://www.jsk.t.u-tokyo.ac.jp/index-j.html"
```

- `:hide-image` (naoqi_bridge [kochigami-develop]/ tablet.launch)

Hide image currently displayed. This method deletes every image, app, webview and shows Pepper's bubbles.  

[ALTabletService::hideImage](http://doc.aldebaran.com/2-5/naoqi/core/altabletservice-api.html?highlight=altablet#ALTabletService::hideImage)  
[related PR](https://github.com/ros-naoqi/naoqi_bridge/pull/52)

```
send *ri* :hide-image
```

***Trouble Shooting***

- How to log in to a robot?
```
ssh nao@<Your robot IP>
```

- How to send a file to a robot?
```
scp <file path of your local PC> nao@<Your robot IP>:/home/nao/.local/share/PackageManager/apps/img/html/
```

- If this kind of error occurs at `tablet.launch`, please re-launch the program. 
```
[ERROR] [1543212108.209135]: Exception caught:
	ALTabletService::showImage
		module destroyed
```

Joints of Pepper
----------------

Here is a list of joints when accessing Pepper.
ex:
```
(send *pepper* :reset-pose)
=> #f(2.0 -2.0 -5.0 85.0 10.0 -70.0 -20.0 -40.0 85.0 -10.0 70.0 20.0 40.0 0.0 0.0)
       0    1    2   3    4     5     6     7    8     9    10   11   12   13  14
```
```
0: :knee-p
1: :hip-r
2: :hip-p
3: :larm :shoulder-p
4: :larm :shoulder-r
5: :larm :elbow-y
6: :larm :elbow-p
7: :larm :wrist-y
8: :rarm :shoulder-p
9: :rarm :shoulder-r
10: :rarm :elbow-y
11: :rarm :elbow-p
12: :rarm :wrist-y
13: :head :neck-y
14: :head :neck-p
```