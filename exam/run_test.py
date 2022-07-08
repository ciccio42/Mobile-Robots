import os 
from datetime import datetime


dict_test = {}

# dict_test[1] = ["1",      "NavfnROS", "DWA"]
# dict_test[2] = ["2",      "NavfnROS", "DWA"]
# dict_test[3] = ["3",      "NavfnROS", "DWA" ]

#dict_test[4] = ["1",      "NavfnROS", "teb_default"]
#dict_test[5] = ["2",      "NavfnROS", "teb_default"]
#dict_test[6] = ["3",      "NavfnROS", "teb_default" ]

#Global DWA, mpc e teb con A*
# dict_test[10] = ["1",      "Global", "DWA"]
# dict_test[11] = ["2",      "Global", "DWA"]
# dict_test[12] = ["3",      "Global", "DWA" ]

# dict_test[13] = ["1",      "Global", "teb"]
# dict_test[14] = ["2",      "Global", "teb"]
# dict_test[15] = ["3",      "Global", "teb" ]

# dict_test[13] = ["1",      "Global_A*_default", "teb"]
# dict_test[14] = ["2",      "Global_A*_default", "teb"]
# dict_test[15] = ["3",      "Global_A*_default", "teb" ]

#dict_test[16] = ["1",      "Global_dijkstra_navguide", "teb_acc_0_5"]
#dict_test[17] = ["2",      "Global_dijkstra_navguide", "teb_acc_0_5"]
#dict_test[18] = ["3",      "Global_dijkstra_navguide", "teb_acc_0_5" ]


# dict_test[19] = ["1",      "NavfnROS_navguide", "teb_acc_0_5_forward_drive_200"]
# dict_test[20] = ["2",      "NavfnROS_navguide", "teb_acc_0_5_forward_drive_200"]
# dict_test[21] = ["3",      "NavfnROS_navguide", "teb_acc_0_5_forward_drive_200" ]

# dict_test[22] = ["1",      "NavfnROS_navguide", "teb_acc_1_5_forward_drive_200"]
# dict_test[23] = ["2",      "NavfnROS_navguide", "teb_acc_1_5_forward_drive_200"]
# dict_test[24] = ["3",      "NavfnROS_navguide", "teb_acc_1_5_forward_drive_200" ]

dict_test[22] = ["1",      "NavfnROS_navguide", "teb_acc_1_5_forward_drive_200"]
dict_test[23] = ["2",      "NavfnROS_navguide", "teb_acc_1_5_forward_drive_200"]
dict_test[24] = ["3",      "NavfnROS_navguide", "teb_acc_1_5_forward_drive_200" ]


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
    bag_name = f"{bag_dir}/{t[0]}_{t[1]}_{t[2]}.bag"
    print(bag_name)
    # print(f"rosbag record -O {bag_dir}/{test}.bag -a ")
    # os.system (f"rosbag record -O {bag_dir}/{t[0]}_{t[1]}_{t[2]}.bag -a -x \"(.*)/(scan|camera)(.*)\" __name:=bag_recorder &")
    print(f"roslaunch exam demo.launch sim:=True num_path:={t[0]} global_planner_param_demo:={t[1]} local_planner_param_demo:={t[2]} bag_name:={bag_name}")
    os.system (f"roslaunch exam demo.launch sim:=False num_path:={t[0]} global_planner_param_demo:={t[1]} local_planner_param_demo:={t[2]} bag_name:={bag_name} run_aip:='True' ")
    log_file.write(f"\nTest {i} completed! [{t[0]} {t[1]} {t[2]}]")
    os.system (f"rosnode kill -a")
    input("Waiting for next test....")

log_file.write(f"\n\nAll test completed")
log_file.close()