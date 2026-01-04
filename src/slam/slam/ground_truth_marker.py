from math import sin, cos, radians

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

PATH_DENSITY = 50 # dot per metre

path_1 = [[0.0,i*0.355/(PATH_DENSITY*0.355),0.0] for i in range(0,int(PATH_DENSITY*0.355))]
path_2 = [[0.0,i*0.21/(PATH_DENSITY*0.21),0.0] for i in range(0,int(PATH_DENSITY*0.21))]
path_2.extend(
    [
        [0.0,0.21,0.0],
        [0.02,0.26,0.0],
        [0.045,0.29,0.0],
        [0.13,0.32,0.0],
        [0.17,0.32,0.0],
        [0.21,0.31,0.0],
        [0.235,0.295,0.0],
        [0.275,0.255,0.0],
    ]
)

map_1 = [
    [(0.13,0.08,0.0), "sphere", (0.09,0.09,0.09), (0.0,0.0,0.0,1.0)],
    [(-0.065,0.175,0.0), "cube", (0.01,0.09,0.1), (0.0,0.0,0.0,1.0)],
    [(0.07,0.225,0.0), "sphere", (0.05,0.05,0.05), (0.0,0.0,0.0,1.0)],
    [(0.08,0.31,0.0), "cube", (0.03,0.06,0.1), (0.0,0.0,sin(radians(45)/2),cos(radians(45)/2))],
    [(-0.1,0.38,0.0), "cube", (0.055,0.28,0.1), (0.0,0.0,sin(radians(135)/2),cos(radians(135)/2))],
]
map_2 = [
    [(0.345,0.35,0.0), "sphere", (0.095,0.095,0.095), (0.0,0.0,0.0,1.0)],
    [(-0.09,0.155,0.0), "cube", (0.01,0.09,0.1), (0.0,0.0,0.0,1.0)],
    [(0.215,0.225,0.0), "sphere", (0.05,0.05,0.05), (0.0,0.0,0.0,1.0)],
    [(0.120,0.42,0.0), "sphere", (0.13,0.09,0.2), (0.0,0.0,0.0,1.0)],
    [(-0.05,0.31,0.0), "cube", (0.03,0.06,0.1), (0.0,0.0,sin(radians(125)/2),cos(radians(125)/2))],
    [(0.105,0.095,0.0), "cube", (0.055,0.28,0.1), (0.0,0.0,sin(radians(10)/2),cos(radians(10)/2))],
]

def publish_ground_truth_map(marker_map, frame_id, publisher, clock):
    marker_array = MarkerArray()
    markers = []

    marker_map = map_1 if marker_map==1 else map_2
    counter = 0

    for obj in marker_map:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = clock.now().to_msg()

        marker.ns = "Ground Truth Path"
        marker.id = counter
        if obj[1] == "sphere":
            marker.type = Marker.SPHERE 
        elif obj[1] == "cube":
            marker.type = Marker.CUBE 
        else:
            marker.type = Marker.SPHERE 

        marker.action = Marker.ADD

        marker.pose.position.x = obj[0][0]
        marker.pose.position.y = obj[0][1]
        marker.pose.position.z = obj[0][2]
        marker.pose.orientation.x = obj[3][0]
        marker.pose.orientation.y = obj[3][1]
        marker.pose.orientation.z = obj[3][2]
        marker.pose.orientation.w = obj[3][3]

        marker.scale.x = obj[2][0]
        marker.scale.y = obj[2][1]
        marker.scale.z = obj[2][2]

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0              # MUST be > 0

        # marker.lifetime.sec = 5           # auto-refresh

        markers.append(marker)
        counter += 1

    marker_array.markers = markers
    publisher.publish(marker_array)


def publish_ground_truth_path(path, frame_id, publisher, clock):
    marker_array = MarkerArray()
    markers = []

    path = path_1 if path==1 else path_2
    counter = 0

    for pose in path:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = clock.now().to_msg()

        marker.ns = "Ground Truth Path"
        marker.id = counter
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # print(pose)

        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = pose[2]
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0              # MUST be > 0

        # marker.lifetime.sec = 5           # auto-refresh

        markers.append(marker)
        counter += 1

    marker_array.markers = markers
    publisher.publish(marker_array)
