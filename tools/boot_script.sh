#! /bin/bash
# Beginning with rosr5 (2018-04) we added an auto reboot capability.
# This routine runs at system reboot to create a folder called ~/tmp
# with a bootlog file
# see https://askubuntu.com/questions/9853/how-can-i-make-rc-local-run-on-startup

# 1) sudo nano /etc/rc.local
# add to /etc/rc.local
#   #!/bin/sh
#   sh '/home/oper/swmain/apps/ROSR/sw/tools/boot_script.sh' -- this file
# 2) sudo chmod 666 /etc/rc.local  -- make it executable


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
