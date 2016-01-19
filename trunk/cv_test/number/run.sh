#!/bin/bash

#python recognize.py --images ../testing_lp_dataset &>run.log
python recognize.py --images ~/sshfs/asus/root/rus_hard/scaremaster &>run.log
echo $?
