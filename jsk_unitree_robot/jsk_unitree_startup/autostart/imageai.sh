#!/bin/bash
Nano1="192.168.123.13"  ## SecNanoRight(1-2)
Nano2="192.168.123.14"	## SecNanoLeft(3-4)
Nano3="192.168.123.15"	## MasterNano(5)
eval echo "[imageai] starting ... " $toStartlog

mLRootPATH="/home/unitree/Unitree/autostart/imageai/"
updatePackagePATH="/home/unitree/mLComSystemFrame.tar.gz"

echo -e "\e[1;32m**** 1. Check if the program needs to be updated ? ****\e[0m"
checkIFUpdate(){
	if [ -f "$updatePackagePATH" ];then
		echo -e "\e[1;32m	The update file exists, it will be updating soon ...\e[0m"
		tar -zxvf $updatePackagePATH -C $mLRootPATH > /dev/null; sleep 1
		rm -rf $updatePackagePATH
		echo -e "\e[1;32m	Updating Success.\e[0m"
	else
		echo -e "\e[1;31m	NO Need to update!\e[0m"
	fi
}

checkIFUpdate
# dd=$?

#declare -i lossNano1=-1
#declare -i lossNano2=-1
#declare -i lossNano3=-1

#echo -e "\e[1;32m**** 2. Check if all Nano'IP Address is OK ? ****\e[0m"

#until (( lossNano1 + lossNano2 + lossNano3 == 0 ))
#do
#	lossNano1=`ping -c 2 -w 2 $Nano1 | grep loss | awk '{print $6}' | awk -F "%" '{print $1}'`
#	echo $lossNano1
#
#	lossNano2=`ping -c 2 -w 2 $Nano2 | grep loss | awk '{print $6}' | awk -F "%" '{print $1}'`
#	echo $lossNano2
#
#	lossNano3=`ping -c 2 -w 2 $Nano3 | grep loss | awk '{print $6}' | awk -F "%" '{print $1}'`
#	echo $lossNano3
#	#sleep 10
#done

#echo "	IP is Right!"

## GET PC's Address
localIPAddr=`ifconfig -a|grep inet|grep -v 127.0.0.1|grep -v inet6| head -n 1 | awk '{print $2}'|tr -d "addr:"`
echo "Local Address: "$localIPAddr

echo -e "\e[1;32m**** 2. Start Main Application ... ****\e[0m"
if [ $localIPAddr == $Nano3 ];then
	echo "	MasterNano"
	############### MasterNano doing things!!!!
	gnome-terminal -- bash -c "export GST_PLUGIN_PATH=/home/unitree/Unitree/autostart/imageai/mLComSystemFrame/ThirdParty/webSinkPipe/build; cd $mLRootPATH/mLComSystemFrame/bin; ./mqttControlNode ../config/mqMNConfig.yaml; exec bash"
	# gnome-terminal -- bash -c "export GST_PLUGIN_PATH=/home/unitree/Unitree/autostart/imageai/mLComSystemFrame/ThirdParty/webSinkPipe/build; cd $mLRootPATH/mLComSystemFrame/pyScripts; python3 live_human_pose.py; exec bash"

elif [ $localIPAddr == $Nano2 ];then
	echo "	SecNanoLeft"
	############### SecNanoLeft(3-4) doing things!!!!
	gnome-terminal -- bash -c "export GST_PLUGIN_PATH=/home/unitree/Unitree/autostart/imageai/mLComSystemFrame/ThirdParty/webSinkPipe/build; cd $mLRootPATH/mLComSystemFrame/bin; ./mqttControlNode ../config/mqSNNLConfig.yaml; exec bash"
	gnome-terminal -- bash -c "export GST_PLUGIN_PATH=/home/unitree/Unitree/autostart/imageai/mLComSystemFrame/ThirdParty/webSinkPipe/build; cd $mLRootPATH/mLComSystemFrame/bin; ./mqttControlNode ../config/mqSNNLConfig.yaml 1; exec bash"

elif [ $localIPAddr == $Nano1 ];then
	echo "	SecNanoRight"
	############### SecNanoRight(1-2) doing things!!!!
	gnome-terminal -- bash -c "export GST_PLUGIN_PATH=/home/unitree/Unitree/autostart/imageai/mLComSystemFrame/ThirdParty/webSinkPipe/build; cd $mLRootPATH/mLComSystemFrame/bin; ./mqttControlNode ../config/mqSNNRConfig.yaml; exec bash"
	gnome-terminal -- bash -c "export GST_PLUGIN_PATH=/home/unitree/Unitree/autostart/imageai/mLComSystemFrame/ThirdParty/webSinkPipe/build; cd $mLRootPATH/mLComSystemFrame/bin; ./mqttControlNode ../config/mqSNNRConfig.yaml 1; exec bash"
  gnome-terminal -- bash -c "export GST_PLUGIN_PATH=/home/unitree/Unitree/autostart/imageai/mLComSystemFrame/ThirdParty/webSinkPipe/build; cd $mLRootPATH/mLComSystemFrame/pyScripts; python3 live_human_pose.py; exec bash"
else
	echo "	Not Found $localIPAddr in IP-Clump!"
fi

echo "	EveryThing is done!"
