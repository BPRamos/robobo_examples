from doepy import build

import sys
import rospy
#import tf2_ros 
from geometry_msgs import Pose, Point, Quaternion
from gazebo_msgs import SpawnModelRequest

def getNewObjPosition(robobo_position_msg, orient_qt, distance):
    robobo_x = 0
    robobo_y = 0
    #get Quaternion
    #quaternion = Quaternion()

    

#rosrun gazebo_ros spawn_model -file ./robobo-gazebo-models/cylinder_medium_blue/model.sdf -sdf -model cylinder_medium_blue -x 0
#/gazebo/spawn_sdf_model
def test_object(spawn_service, object_name, object_xml, x_list):
    init_x = 0
    init_y = 0
    init_z = 0

    init_position = Point(init_x, init_y, init_z)
    init_orientation = Quaternion(0,0,0,0)
    init_pose = Pose(init_position, init_orientation)
    spawn_service.spawn_sdf_model(mode_name=object_name, model_xml=object_xml, initial_pose=init_pose)

def get_service_client(service_name, service_type):
    rospy.wait_for_service(service_name)
    try:
        service = rospy.ServiceProxy(service_name, service_type)
        return service
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def main():
    try:
        espacial = build.full_fact(
        {'Distancia (cm)':[10,30,50],
        'Pan (deg)':[0,30,60],
        'Tilt (deg)':[70,90,110],}
        ).sort_values(['Distancia (cm)','Tilt (deg)','Pan (deg)'],ignore_index=True)
        print(espacial)

        illum = build.full_fact(
        {'Ilum (bool)':[0,1],
        'Pan (deg)':[0,15,30,60],
        'Distancia (cm)':[10,30,50]}
        ).sort_values(['Distancia (cm)','Ilum (bool)','Pan (deg)'],ignore_index=True)
        print(illum)

        #result = service_client()
    except rospy.ROSInterruptException:
        print("Shutting down Robobo Experiment module")

if __name__ == '__main__':
    main()