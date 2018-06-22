#! /bin/bash
# see https://askubuntu.com/questions/9853/how-can-i-make-rc-local-run-on-startup
# add to /etc/rc.local
#    add sh /home/oper/swmain/apps/ROSR/sw/tools/boot_script.sh
# Beginning with rosr5 (2018-04) we added an auto reboot capability.
# This routine runs at system reboot to create a folder called ~/tmp
# with a bootlog file

  # edit 20180509T165124Z
  # REBOOT FOLDER ~/tmp
  # Create if it does not exist
file="/home/oper/tmp";
if [ ! -d $file ]
then
  mkdir $file
fi
touch /home/oper/tmp/AutoStartFlag
echo boot_script -- create AutoStartFlag >> /home/oper/tmp/bootlog
chmod 666 /home/oper/tmp/AutoStartFlag

exit 0
