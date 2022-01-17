#!/bin/bash
#set -e

printf "Building workspace...\n\n"
colcon build --symlink-install --event-handlers console_direct+

printf "\nSourcing install/local_setup.bash\n"
source install/local_setup.bash
#printf "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH\n"
