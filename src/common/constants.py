class Constants:
    # ROS2 node naemes
    SUBSCRIBER_NODE_NAME = "slam_subscriber"
    PUBLISHER_NODE_NAME = "slam_publisher"

    # Topic names when using EventBus
    EVENT_TOPIC_MOVEMENT = "robot_output"

    # Topic names belonging to Robile
    ROS_TOPIC_MAP = "/map"
    ROS_TOPIC_ODOM = "/odom"
    ROS_TOPIC_LIDAR = "/scan"
    ROS_TOPIC_MOVEMENT = "/cmd_vel"

    # Constants used by path planning services
    MAX_LINEAR_VELOCITY = 2.0
    MAX_ANGULAR_VELOCITY = 1.0
    ATTRACTIVE_GAIN = 2.0
    REPULSIVE_GAIN = 0.01
    REPULSIVE_DISTANCE = 0.5
