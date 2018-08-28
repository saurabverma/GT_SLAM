# GT_SLAM (Ground Truth SLAM)

This is an effort to create a robust backend SLAM which can input data from different sensors and out the best possible quality of map generated.
Loop closures are also handled by the system in both automatic and manual approaches.
The system is designed to be used in offline mode because of a lot of backend optimisation processes running based on constraint-requirements.

These ROS packages are designed by combining only required extracts from a lot of well-tested online available packages such as Blam-Slam, LOAM, etc.

## Input

1. BAG FILE: A rosbag file containing a laser scan time-stamped data.
2. PBSTREAM: Atleast one pbstream data contaning time-stamped pose of the robot estimated from different sensors (the time stamps need not be the same as that of laser scan data).

## OUTPUT:

1. PBSTREAM: (Optional) A pbstream file containing optimized poses of the robot for each key-frame laser scan data.
The optimized/corrected ground truth map can be manually regenerated later at any time using respective time-stamped poses from this output file.

## Compilation

Run the file: "./update" to compile all required packages.

## Run

Call the "LOAM_loopClose_offline.launch" to start the process.

NOTE: The tuning parameters are available in yaml files of the respective packages.

## Dependencies

NOTE: BLAM relies on system installations of GTSAM (https://collab.cc.gatech.edu/borg/gtsam).
GTSAM in particular should be installed manually from source using the latest version of the develop branch from https://bitbucket.org/gtborg/gtsam.

The package is tested to be functional with ROS Kinetic and Ubuntu 16.04.

For further details, please contact me:
saurabverma@u.nus.edu; saurabverma@gmail.com

