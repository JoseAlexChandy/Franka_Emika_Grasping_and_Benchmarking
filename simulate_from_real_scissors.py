import os

import rospy
from geometry_msgs.msg import Pose

from grasp_benchmark.utils.ros_utils import *
from alr_sim.sims.SimFactory import SimRepository


from utils.ycb_utils import YCBLoader

NUMBER_OF_CANDIDATES = 1

global_scene = None

def main():
    # ROS Init
    rospy.init_node("test")
    pub_grasp_pose = rospy.Publisher('grasp_pose', Pose, queue_size=10)
    grasp_pose = Pose()

    factory_string = "mj_beta"
    YCB_loader = YCBLoader(ycb_base_folder="/home/heap/GraspBenchmarkWorkspace/YCB/models/ycb/", factory_string=factory_string)
    
    print('waiting for pose of the brick')
    pose_msg = rospy.wait_for_message('brick_pose', Pose)
    print(pose_msg)
    q_WB = [pose_msg.orientation.w, 
            pose_msg.orientation.x, 
            pose_msg.orientation.y,
            pose_msg.orientation.z]
    rBW_W = [pose_msg.position.x, pose_msg.position.y, -0.017]
    print('received position:', rBW_W)
    print('received orientation:', q_WB)

    scissors, scissors_name = YCB_loader.get_ycb_object(obj_id="037_scissors", 
                                                    pos=rBW_W, 
                                                    quat=q_WB)
    #foam_brick, foam_brick_name = YCB_loader.get_ycb_object(obj_id="061_foam_brick", 
    #                                                        pos=rBW_W, 
    #                                                        quat=q_WB)
    #apple, apple_name = YCB_loader.get_ycb_object(obj_id="013_apple", 
    #                                                pos=[1, 0.2, -0.017], 
    #                                                quat=[1.0, 0, 0, 0])

    # Initializing
    #object_list = [foam_brick, apple]
    object_list = [scissors]
    
    
    # Generate the chosen Scene and Agent
    sim_factory = SimRepository.get_factory(factory_string)
    scene = sim_factory.create_scene(object_list=object_list, dt=0.0003)
    agent = sim_factory.create_robot(scene, dt=0.0003)

    agent.inhand_cam.set_cam_params(width=640, height=480)

    scene.start()
    #print('brick position')
    #print(scene.get_obj_pos(obj_name = foam_brick_name))
    #print('apple position')
    #print(scene.get_obj_pos(obj_name = apple_name))
    print('scissors position')
    print(scene.get_obj_pos(obj_name = scissors_name))


    

    target_obj_id = scene.get_obj_seg_id(obj_name=scissors_name)
    #print('for apple:')
    #print(scene.get_obj_seg_id(obj_name=apple_name))
    #print('for brick:')
    #print(scene.get_obj_seg_id(obj_name=foam_brick_name))
    print('for scissors:')
    print(scene.get_obj_seg_id(obj_name=scissors_name))
    #print(scene.get_obj_id(obj_name=foam_brick_name))


    # Currently the throw position is the same for all objects
    throw_pos, throw_quat = [0.776813140, 0.0, 0.200013516], [0, 1, 0, 0]

    # 1. Going to home for the start
    home_pos, home_quat = [0.32, 0.0, 0.56], [0, 1, 0, 0]
    agent.gotoCartPositionAndQuat(home_pos, home_quat)
    agent.open_fingers()
    # Get the cam for intrinsics and other info

    #print('final brick position')
    #print(scene.get_obj_pos(obj_name = foam_brick_name))
    #print('final apple position')
    #print(scene.get_obj_pos(obj_name = apple_name))
    print('final scissors position')
    print(scene.get_obj_pos(obj_name = scissors_name))

    print('----------------------------')
    print('select grasping algorithm:\n1. DEXNET \n2. GRASPNET \n3. GPD\n')
    algorithm_number = input()
    if int(algorithm_number) == 1:
        print('calling Dexnet...')
        algorithm_name = "/dexnet_bench/dexnet_grasp_planner_service"
    elif int(algorithm_number) == 2:
        print('calling Graspnet...')
        algorithm_name = "/graspnet_bench/graspnet_grasp_planner_service"
    else: 
        print('calling GPD...')
        algorithm_name = "/gpd_bench/gpd_grasp_planner/gpd_grasp_planner_service"

    # 2. Calling the grasp planner
    result_from_service = call_grasp_planner(cam=agent.inhand_cam, 
                                            service_id=algorithm_name, 
                                            seg_obj_id=target_obj_id, 
                                            num_of_candidates=NUMBER_OF_CANDIDATES)
    print(result_from_service)
    pos = result_from_service[0]['position']
    quat = result_from_service[0]['quat'] 


    
    # pos[2] += 0.01

    # 3. Moving shortly above the planned grasp
    agent.gotoCartPositionAndQuat([pos[0], pos[1], pos[2] + 0.1], quat, duration=6)
    agent.wait(1)

    # 4. Going to the grasp position
    agent.gotoCartPositionAndQuat(pos, quat, duration=2)
    agent.wait(1)

    # 5. Grasping
    agent.close_fingers()
    agent.wait(1)

    # 6. Move back up
    agent.gotoCartPositionAndQuat([pos[0], pos[1], pos[2] + 0.1], quat, duration=2)
    agent.wait(1)

    # 7. Going to the throw position
    #agent.gotoCartPositionAndQuat(throw_pos, throw_quat)
    #agent.wait(1)

    # 7. Letting it fall
    agent.open_fingers()

    to_real = input('execute in real robot? (y/n)')
    if to_real == 'y':
        grasp_pose.position.x = pos[0] 
        grasp_pose.position.y = pos[1] 
        grasp_pose.position.z = pos[2]+(0.017-0.005) #compensation due to table level
        grasp_pose.orientation.w = quat[0]
        grasp_pose.orientation.x = quat[1]
        grasp_pose.orientation.y = quat[2]
        grasp_pose.orientation.z = quat[3]
        for i in range(100):
            pub_grasp_pose.publish(grasp_pose)


if __name__ == "__main__":
    main()



# SCRIPT ON BOB:
#   conda activate grasp_benchmark3
#   source ~/catkin_ws/devel/setup.bash
#   python ~/GraspBenchmark/Benchmark/grasp_benchmark/simulate_from_real.py
