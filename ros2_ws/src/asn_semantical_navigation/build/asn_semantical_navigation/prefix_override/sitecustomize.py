import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/master-jan/projects/autonomous-semantic-navigation/ros2_ws/src/asn_semantical_navigation/install/asn_semantical_navigation'
