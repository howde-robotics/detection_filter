# detection_filter
Node to filter raw human detection locations with the Wire package

# Human Detection Pipeline
Here is how the human detection pipeline currently works:
![Human Detection Pipeline](graphic.svg)

1. The `darknet` node pushes out bounding boxes of detected objects. 
1. These bounding boxes are captured by the `rgb_human_detection_node` along with near-syncrhonous depth images. Image transofmrations are used to rectify the bounding boxes into the depth frame, from which a depth value is extracted, and projected into the real world relative to the RealSense camera. These *relative human locations* in (x,y,z) are then pushed out as `/dragoon_messages/Objects`.
2. This node (`detection_filter`) grabs these *relative human detection locations* and pre-processes them (April 12: Pre-processesing only consists of ignoring relative detections over 8m away or under 0.05m away). These *pre-processed detections* are then transformed to the `/map` frame using the TF-tree and are published as `wire_msgs/WorldEvidence` to be fed into the global Kalman filter.
3. The global Kalman filter `wire_core` takes in these `wire_msgs/WorldEvidence` evidences, does probability magic, and outputs `wire_msgs/WorldState` messages representing our *filtered human detections* in the `/map` frame. 
4. The visualizer eats up `wire_msgs/WorldEvidence` and `wire_msgs/WorldState` to display markers that represent the world evidence (*pre-processed detections* transformed to the global frame) and the world state (aka *filtered detections*) in their appropriate places on the `/map` frame. 

# Working with our new rosbag
So we clearly have some issues with the human detection pipeline. Where are they? Fuck me I don't know but lets find out. Something is certainly wrong in the `rgb_human_detection_node`, `detection_filter`, and/or `visualizer`, or some synchronization/timing issue between imagery. Testing out an updated node anywehere along this chain requires removing some of the messages from the rosbag above. Specifically, it requires removing all messages along this pipeline downstream of the targeted node. 

That said, the best way to try out new algorithms is the folliwing procedure:

1. Remove all rosmsgs downstream of the `darknet` node
2. Run all downstream nodes locally
3. Replay the rosbag with `use_sime_time` set to true and `--clock` set to the rosbag timer

## Removing downstream msgs

Use this command to filter the new bag into a filtered one called `April12_Filtered.bag` or whatever the fuck you want I'm not your dad

```bash
rosbag filter April12_SVD_test_human_detection_tf_off.bag April12_Filtered.bag "topic!='/world_state' and topic!='/world_evidence' and topic!='/ObjectPoses'"
```

## Running downstream nodes locally
1. Pull our repos for `darkenet_ros_msgs` (see last section for only pulling this), `RealSenseDev` (which includes `rgb_human_detection_node`), `detection_filter`, `wire` and `visualizer` into you `catkin_ws/src/`. If you havent already installed jsk for the visualizer then: `sudo apt-get install ros-melodic-jsk-visualization`. There might be other dependencies just hmu if you run into issues building these nodes. `wire` is gonna throw a fuck ton of warnings when you build about some highly problematic `Cube`, idk what they are but don't worry about them.
2. For the love of god download terminator because you're about the have like 30 terminals open


## Just getting `darknet_ros_msgs`
If you just want the messages and not having to pull the whole darknet repo with all the weights and shit:

```bash
cd catkin_ws/src/
mkdir darknet_ros
cd darknet_ros
git init
git remote add origin https://github.com/howde-robotics/darknet_ros.git
git fetch
git checkout origin/master darknet_ros_msgs/*
```

