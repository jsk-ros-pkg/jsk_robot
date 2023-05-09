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
rosrun aibo_selenium_ros main.py _webdriver:=<path to chromedriver>
```

This script shows up a selenium controlled webbrowser window.

<TODO Image>

Please sign in to myaibo manually and open the dashboard page below.

<TODO Image>

Then hit Enter key on the `main.py` window. So it will start to control browser and publish image.

<TODO Movie>

## How to run (Auto-login mode)

You may be able to use `auto_login` mode.

```bash
rosrun aibo_selenium_ros main.py _webdriver:=<path to chromedriver> _login_id:=<login_id> _login_password:=<login_password> _auto_login:=true
```

You can also use headless mode. by setting parameter.

```bash
rosrun aibo_selenium_ros main.py _webdriver:=<path to chromedriver> _login_id:=<login_id> _login_password:=<login_password> _auto_login:=true _headless:=true
```
