# aibo_selenium_ros

This package provides a ROS Node which retieve head camera images from web browser and publish it as ROS message.

## How to setup

**Currently this node is only tested with ROS noetic.**

First, Install [Google chrome](https://www.google.com/intl/ja_jp/chrome/) to your PC.

Second, Download [suitable version of chrome webdriver to Google chrome installed](https://chromedriver.chromium.org/downloads) and place it somewhere.

And install pip dependencies

```bash
cd <directory of aibo_selenium_ros>
pip install -r .
```

Build

```bash
catkin bt
```

## How to run (Manual mode)

To run this node, first run roscore

```bash
roscore
```

Then run `main.py`

```bash
$ rosrun aibo_selenium_ros main.py _webdriver:=<path to chromedriver>

Press Enter when logging if completed.
```

This script shows up a selenium controlled webbrowser window.

![aibo_login](https://user-images.githubusercontent.com/9410362/237017652-29c64750-a3a6-4008-a197-9c2cc5ba5bdb.png)

Please sign in to myaibo manually and open the dashboard page below.

![dashboard](https://user-images.githubusercontent.com/9410362/237017806-9f85e696-1f1c-4362-b1cc-22b0fe0e7635.png)

Then hit Enter key on the `main.py` window. So it will start to control browser and publish image.

![output](https://user-images.githubusercontent.com/9410362/237018221-13d6a118-6ec2-44ff-86eb-eecf75cfe218.gif)


## How to run (Auto-login mode)

You may be able to use `auto_login` mode. **This mode is unstable**

```bash
rosrun aibo_selenium_ros main.py _webdriver:=<path to chromedriver> _login_id:=<login_id> _login_password:=<login_password> _auto_login:=true
```

You can also use headless mode. by setting parameter.

```bash
rosrun aibo_selenium_ros main.py _webdriver:=<path to chromedriver> _login_id:=<login_id> _login_password:=<login_password> _auto_login:=true _headless:=true
```
