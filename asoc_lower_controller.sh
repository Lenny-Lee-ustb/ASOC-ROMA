#!/bin/bash

roslaunch SpimTest asoc_encoder.launch &
sleep 2
echo "encoder startgin success!"

roslaunch asoc_lower_controller asoc_lower_controller.launch &
sleep 0.1
echo "asoc_lower_controller starting success!"
wait
exit 0
