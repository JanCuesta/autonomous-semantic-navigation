V3 NBV RViz config overlay. Expects the Step 3 asn_semantical_navigation package to be already installed.

Unzip at project repository root, rebuild both asn_semantical_navigation and asn_bringup.
The existing v3_autonomous.launch.py should point to asn_semantical_navigation/rviz/Semantical_nav_map.rviz.
By default the predicted map, best pose arrow, and depth point cloud are disabled, but can be enabled from the Displays panel.
There are 12 candidate markers, 8 positional + 4 rotation. Invalid=gray, alternatives=blue, selected=green.
Published NBV markers are volatile reliable; coverage map and occupancy map are transient local reliable.
The RViz file only changes visualization; it does not issue robot commands.
