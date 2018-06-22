#!/bin/bash -e
#
# rc.local
#
#This script is copied to /etc/ for rmrco daq software.
#!! rename to from ROSR/sw/tools/ "sudo cp rc.local.sh /etc/rc.local"
 
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.

# see
# https://askubuntu.com/questions/9853/how-can-i-make-rc-local-run-on-startup

echo rc.local boot >> /home/oper/tmp/bootlog

bash /home/oper/swmain/apps/ROSR/sw/tools/boot_script.sh

exit 0
