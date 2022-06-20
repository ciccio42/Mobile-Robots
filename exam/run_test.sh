roslaunch exam demo.launch sim:=True num_path:=3 
rosnode kill -a
echo "primo test done"
roslaunch exam demo.launch sim:=True num_path:=1 
rosnode kill -a
echo "secondo test done"