# rospy-bayes-filter

## Files for ROS PA3
- src/tagGenerator.py: Separate node for publishing the cubes for landmarks
- src/rviz\_utils.py: Simple utility file for publishing markers to RViz based on provided example markers file
- src/rosbagParser.py: Actual node that performs the Bayes filter execution, after reading ROS Bag
- launch/pa3.launch: Launch file that properly launches all nodes
- include/config.rviz: Configuration file for RViz that initializes RViz properly for PA3 submission
