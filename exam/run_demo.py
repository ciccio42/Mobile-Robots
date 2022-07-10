import os 
from datetime import datetime
import argparse
 
parser = argparse.ArgumentParser(description="Navigation Node parameters",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("-path_number", "--path-number", default=None, help="Number of the path")
args, unknown = parser.parse_known_args()

GLOBAL_PLANNER = "NavfnROS_navguide"
LOCAL_PLANNER = "teb_acc_1_5_forward_drive_200_best"

# create log file
now = datetime.now()
dt_string = now.strftime("%d%m%Y_%H:%M")
folder = dt_string.split("_")[0]
name = dt_string.split("_")[1]
log_directory = "log_test/"+str(folder)
if not os.path.exists(log_directory):
    os.makedirs(log_directory)
log_file = open(str(log_directory)+"/"+str(name)+".txt","w") 


print(f"roslaunch exam demo.launch sim:=False num_path:={args.path_number} global_planner_param_demo:={GLOBAL_PLANNER} local_planner_param_demo:={LOCAL_PLANNER} ")
os.system (f"roslaunch exam demo.launch sim:=False num_path:={args.path_number} global_planner_param_demo:={GLOBAL_PLANNER} local_planner_param_demo:={LOCAL_PLANNER} run_aip:='True' ")
log_file.write(f"\nTest completed! [{args.path_number} {GLOBAL_PLANNER} {LOCAL_PLANNER}]")
os.system (f"rosnode kill -a")

log_file.write(f"\n\nAll test completed")
log_file.close()