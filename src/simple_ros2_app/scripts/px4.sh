#!/bin/bash
SIM_DIR="$HOME/.ros/sitl_iris_$1"
mkdir -p "$SIM_DIR"

cat << EOF > "$SIM_DIR/custom_rc.sh"
#!/bin/sh
. etc/init.d-posix/rcS

param set COM_RC_IN_MODE 1
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
param set SIM_BAT_DRAIN 0
param set COM_ARM_BAT_MIN 0

EOF

chmod +x "$SIM_DIR/custom_rc.sh"
cd ~/.ros

# Check if using ROS2026 path or the other one. Assuming the original path from the script works.
~/ROS/PX4-Autopilot/build/px4_sitl_default/bin/px4 ~/ROS/PX4-Autopilot/build/px4_sitl_default/etc -s "$SIM_DIR/custom_rc.sh" -i $1 -w "$SIM_DIR"

exit 0