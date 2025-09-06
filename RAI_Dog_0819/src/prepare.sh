#!/bin/bash

sudo chmod 666 /dev/tty*USB*
ls -l /dev/tty*USB*
python3 Check_detector.py
echo "Usage: nohup python3 Run_crossing1.py eth0 &
    or nohup python3 Run_crossing2.py eth0 &"

