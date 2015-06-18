#!/bin/bash

template_desktop_files=`rospack find jsk_baxter_desktop`/desktop_shortcut_template/*

#Ask robot name
echo "Enter Baxter Name:"
read BAXTER_NAME
icon_base_directory=`rospack find jsk_baxter_desktop`/icons
script_base_directory=`rospack find jsk_baxter_desktop`/scripts/desktop_scripts
mkdir -p $script_base_directory/tmp

for template_desktop_file in $template_desktop_files; do
    echo $template_desktop_file;
    template_base_filename=${template_desktop_file##*/};
    desktop_file_without_extension=${template_base_filename%.*}
    
    cp $template_desktop_file $script_base_directory/tmp/$desktop_file_without_extension

    sed -i -e "s|ICON_BASE_DIRECTORY|$icon_base_directory|g" $script_base_directory/tmp/$desktop_file_without_extension;
    sed -i -e "s|SCRIPT_BASE_DIRECTORY|$script_base_directory|g" $script_base_directory/tmp/$desktop_file_without_extension;
    sed -i -e "s|BAXTER_ROBOT_NAME|$BAXTER_NAME|g" $script_base_directory/tmp/$desktop_file_without_extension;
    sed -i -e "s|ROS_DISTRO|$ROS_DISTRO|g" $script_base_directory/tmp/$desktop_file_without_extension;
    chmod 755 $script_base_directory/tmp/$desktop_file_without_extension;
    cp $script_base_directory/tmp/$desktop_file_without_extension $HOME/Desktop/$desktop_file_without_extension;
done

rm -rf $script_base_directory/tmp
