#! /bin/bash

###############################################################################
# 1.add Additional startup programs
# start_rosmaster_app
# bash /home/jetson/Rosmaster/rosmaster/start_app.sh
# start app program
###############################################################################


gnome-terminal -- bash -c "sleep 3;python3 /home/jetson/Rosmaster/rosmaster/rosmaster_main.py;exec bash"

wait
exit 0
