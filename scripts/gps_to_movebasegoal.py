import rospy
import actionlib
import sys
from geographic_msgs.msg import GeoPoint
from robot_localization.srv import FromLL
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Quaternion
import tf2_ros


class GPS_NavTarget_Interface:
    def __init__(self):
        self.initial_pose = None

        rospy.init_node('gps_nav_goal')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Service to convert GPS (LL) to Pose
        rospy.wait_for_service('fromLL')
        self.from_ll_service = rospy.ServiceProxy('fromLL', FromLL)

    def send_gps_goal(self, latitude, longitude, altitude):
        try:
            gps_point = GeoPoint()
            gps_point.latitude = latitude
            gps_point.longitude = longitude
            gps_point.altitude = altitude

            # Convert GPS to pose
            response = self.from_ll_service(gps_point)
            # Log the converted pose
            rospy.loginfo("Converted pose: %s", response.map_point)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "odom"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position = response.map_point
            goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)  # Default orientation (facing forwards)

            # Send the goal
            self.client.send_goal(goal)
            self.client.wait_for_result()

            return self.client.get_result()

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: gps_nav_goal.py latitude longitude altitude")
        sys.exit(1)

    gps_interface = GPS_NavTarget_Interface()
    latitude = float(sys.argv[1])
    longitude = float(sys.argv[2])
    altitude = float(sys.argv[3])
    result = gps_interface.send_gps_goal(latitude, longitude, altitude)
    print("Navigation result:", result)
