# Mobile Robot for Critial Mission: Final Exam

Delivered by:
* Francesco Rosa, 8802000026

## Folder content

* **exam**: this folder contains the implemented ROS package. 
    * **source**: sub-folder which contains the file *main.py* that is the code entry point
    * **scripts**: sub-folder which contains auxiliary file such as:
        * *Kalman_Filter.py*,
        * *line_extractor.py*,
        * *utils.py*, file that contains some auxiliary function and variables definitions
    * **material**: sub-folder which contains the papers used as guidelines
    * **param**: sub-folder which contains the configuration parameters of the different plug-in, regarding both Global Planner and Local Planner. For the delivered configuration refers to *global_planner_params_NavfnROS_navguide.yaml* and *local_planner_params_teb_acc_1_5_forward_drive_200_best.yaml*. 
    * **config**: sub-folder which contains the paths used for testing the proposed configuration

## How to run the code
1. Copy the **exam** folder in the src folder of your workspace
2. Buil and source
3. Run the commands below 
```bash
cd src/exam
source run_demo.sh [PATH_NUMBER]
```
## Note
To add a new path follow the format given in the examples, and save the file in the **config** directory with name *path_[NUMBER].csv*

