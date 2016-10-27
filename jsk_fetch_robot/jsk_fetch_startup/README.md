# jsk_fetch_startup

## SetUp (Running following commands in the first time)


### upstart
```
su -c 'rosrun jsk_fetch_startup install_upstart.sh'
```

### mongodb

```
sudo mkdir -p /var/lib/robot/mongodb_store/

# to see the db items from http://lcoalhost/rockmongo
sudo apt-get install apache2 libapache2-mod-php5 php5-mongo
wget "http://rockmongo.com/downloads/go?id=14" -O rockmongo.zip
unzip rockmongo.zip
sudo mv rockmongo-1.1.7 /var/www/html/rockmongo
# manually change following line in /var/www/html/rockmongo/config.php
# $MONGO["servers"][$i]["control_auth"] = false; // true;//enable control users, works only if mongo_auth=false
```

## Maintenance

### re-roslaunch jsk_fetch_startup fetch_bringup.launch
```
sudo service jsk-fetch-startup restart
```
as of 2016/10/26, it uses launch files under `~k-okada/catkin_ws`

### re-roslaunch fetch_bringup fetch.launch
```
sudo service robot restart
```

### [Clock Synchronization](https://github.com/fetchrobotics/docs/blob/0c1c63ab47952063bf60280e74b4ff3ae07fd914/source/computer.rst)

install `chrony` and add ```server `gethostip -d fetch15` offline minpoll 8``` to /etc/chrony/chrony.conf, restart chronyd by `sudo /etc/init.d/chrony restart` and wait for few seconds, if you get
```
$ chronyc tracking
Reference ID    : 133.11.216.145 (fetch15.jsk.imi.i.u-tokyo.ac.jp)
Stratum         : 4
Ref time (UTC)  : Wed Oct 26 12:32:56 2016
System time     : 0.000006418 seconds fast of NTP time
Last offset     : 0.003160040 seconds
RMS offset      : 0.003160040 seconds
Frequency       : 11.749 ppm fast
Residual freq   : -137.857 ppm
Skew            : 6.444 ppm
Root delay      : 0.185045 seconds
Root dispersion : 0.018803 seconds
Update interval : 2.1 seconds
Leap status     : Normal
```
it works, if you get `127.127.1.1` for `Reference ID`, something wrong


## Administration

- 2016/10/26 add `allow 133.11.216/8` to /etc/chrony/chrony.conf
```
