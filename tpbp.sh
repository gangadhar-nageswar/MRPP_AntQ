xterm -e "roscore" &
sleep 3
xterm -e "rosparam load $1" &
sleep 2
xterm -e "rosrun mrpp_sumo sumo_wrapper.py" &
sleep 3
xterm -e "rosrun mrpp_sumo $2.py" &
sleep 2
xterm -e "rosrun mrpp_sumo command_center.py" 
sleep 3
killall -w xterm & sleep 5
