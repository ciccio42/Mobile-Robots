#/bin/bash
echo "source ../../devel/setup.bash"
source ../../devel/setup.bash

echo "python3 run_demo.py"
echo "Pat number $1" 
python3 run_demo.py -path_number $1

