import os 
from datetime import datetime

os.system("source ../../devel/setup.bash")

dict_test = {}

dict_test[1] = ["1",      "NavfnROS", "DWA"]
# dict_test[2] = ["2",      "NavfnROS", "DWA"]
#dict_test[3] = ["3",      "NavfnROS", "DWA" ]
# dict_test[4] = ["test_1", "NavfnROS", "DWA" ]
# dict_test[5] = ["test_2", "NavfnROS", "DWA" ]

# dict_test[6] = ["1",      "NavfnROS", "teb"]
# dict_test[7] = ["2",      "NavfnROS", "teb"]
# dict_test[8] = ["3",      "NavfnROS", "teb" ]
# dict_test[9] = ["test_1", "NavfnROS", "teb" ]
# dict_test[10] =["test_2", "NavfnROS", "teb" ]

# dict_test[11] = ["1",      "NavfnROS", "mpc"]
# dict_test[12] = ["2",      "NavfnROS", "mpc"]
# dict_test[13] = ["3",      "NavfnROS", "mpc" ]
# dict_test[14] = ["test_1", "NavfnROS", "mpc" ]
# dict_test[15] = ["test_2", "NavfnROS", "mpc" ]

# dict_test[16] = ["1",      "NavfnROS", "base"]
# dict_test[17] = ["2",      "NavfnROS", "base"]
# dict_test[18] = ["3",      "NavfnROS", "base" ]
# dict_test[19] = ["test_1", "NavfnROS", "base" ]
#dict_test[20] = ["test_2", "NavfnROS", "base" ]

# create log file
now = datetime.now()
dt_string = now.strftime("%d%m%Y_%H:%M")
folder = dt_string.split("_")[0]
name = dt_string.split("_")[1]
log_directory = "log_test/"+str(folder)
if not os.path.exists(log_directory):
    os.makedirs(log_directory)
log_file = open(str(log_directory)+"/"+str(name)+".txt","w") 

bag_dir = "bag/"+str(folder)
if not os.path.exists(bag_dir):
    os.makedirs(bag_dir)

for i, test in enumerate(dict_test.keys()):
    t =  dict_test[test]
    print(f"rosbag record -O {bag_dir}/{test}.bag -a ")
    os.system (f"rosbag record -O {bag_dir}/{test}.bag -a &")
    print(f"roslaunch exam demo.launch sim:=True num_path:={t[0]} global_planner_param_demo:={t[1]} local_planner_param_demo:={t[2]}")
    os.system (f"roslaunch exam demo.launch sim:=True num_path:={t[0]} global_planner_param_demo:={t[1]} local_planner_param_demo:={t[2]}")
    log_file.write(f"\nTest {i} completed! [{t[0]} {t[1]} {t[2]}]")
    os.system (f"rosnode kill -a")

log_file.write(f"\n\nAll test completed")
log_file.close()
