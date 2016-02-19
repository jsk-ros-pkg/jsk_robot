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

