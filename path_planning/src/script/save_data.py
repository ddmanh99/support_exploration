import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import os 
import yaml

yaml_file = '/home/manh/exploration/src/uav_frontier_exploration_3d/config/kopterworx_exploration_sim_large_env.yaml'

f = open(yaml_file, "r")
data = yaml.safe_load(f)
f.close()

odomSave_path = data["octomap"]["file_path"] + data["octomap"]["filename"] + "_odom.txt"

# print (odomSave_path)

class SaveData():
    def __init__(self):
        self.save_path = odomSave_path

        print(f"Save file in {self.save_path}")

        if os.path.exists(self.save_path):
            os.remove(self.save_path)

        self.rb_Odometry_topic = rospy.Subscriber("/hummingbird/ground_truth/odometry_throttled",Odometry,self.rbOdometry_cb)

        self.rbOdometry = Odometry()

        rospy.on_shutdown(self.info)
    
    def rbOdometry_cb(self, msg):
        self.rbOdometry = msg
    
    def write_data(self):
        with open(self.save_path, 'a') as a:
            a.write(str(self.rbOdometry.header.stamp.nse) + " " +
                    str(self.rbOdometry.pose.pose.position.x) + " " + 
                    str(self.rbOdometry.pose.pose.position.y) + " " + 
                    str(self.rbOdometry.pose.pose.position.z) + "\n")
    
    def info(self):
        rospy.loginfo(f'\033[94mData result saved in\033[0m {self.save_path}')

if __name__ == "__main__":
    rospy.init_node('save_data',anonymous=False)
    rate = 5
    r = rospy.Rate(rate)
    rospy.loginfo("\033[92mInit Node Saving positon ...\033[0m")

    s = SaveData()

    while not rospy.is_shutdown():
        s.write_data()
        r.sleep()
    