import os
import sys
import rospy
from geometry_msgs.msg import Pose
from grasp_benchmark.utils.ros_utils import *
from alr_sim.sims.SimFactory import SimRepository
from utils.ycb_utils import YCBLoader
import pywavefront

#For other object or if for example: "Banana" Run it using the command  python your_script_name.py "011_banana" "model_dir"

# Set the object ID here or pass it as a command line argument
base_dir = "/home/heap/GraspBenchmarkWorkspace/YCB/models/ycb/" # You can chane the model directory here or give it as a parameter when you run the script.
object_id = "006_mustard_bottle"  # default object

if len(sys.argv) > 1:
    object_id = sys.argv[1]
    base_dir = sys.argv[2]

NUMBER_OF_CANDIDATES = 1
global_scene = None

def calculate_dimensions(vertices):
    x_min, x_max = np.min(vertices[:, 0]), np.max(vertices[:, 0])
    y_min, y_max = np.min(vertices[:, 1]), np.max(vertices[:, 1])
    z_min, z_max = np.min(vertices[:, 2]), np.max(vertices[:, 2])
    return x_max,x_min, y_max,y_min, z_max,z_min

def load_mesh_model(file_path):
    scene = pywavefront.Wavefront(file_path)
    items = list(scene.materials.items())
    ver = items[0][1].vertices
    n_ver = int(len(ver)/8)
    v = np.zeros((n_ver,3))
    for i in range(n_ver):
        v[i,:] = ver[8*i+5:8*i+5+3]
    return v
    
def main():
    # ROS Init
    rospy.init_node("test")
    pub_grasp_pose = rospy.Publisher('grasp_pose', Pose, queue_size=10)
    grasp_pose = Pose()

    factory_string = "pybullet"
    YCB_loader = YCBLoader(ycb_base_folder = base_dir, factory_string=factory_string)
    # List all directories in the base path
    model_dirs = [d for d in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, d))]
    # Construct the file path for each model
    for model_dir in model_dirs:
        if model_dir == object_id:
            #input("Success")
            file_path = os.path.join(base_dir, model_dir, 'google_16k', 'textured.obj')
            #OUTPUT_PATH = f'{out_path}{os.path.basename(os.path.dirname(os.path.dirname(file_path)))}.png'
            if os.path.isfile(file_path):
                vertices = load_mesh_model(file_path)
                x_max,x_min, y_max,y_min, z_max,z_min = calculate_dimensions(vertices)
    
    print('waiting for pose of the object')
    pose_msg = rospy.wait_for_message('object_pose', Pose)
    q_WB = [pose_msg.orientation.w, 
            pose_msg.orientation.x, 
            pose_msg.orientation.y,
            pose_msg.orientation.z]
    rBW_W = [pose_msg.position.x, pose_msg.position.y, z_min]
    print("pose_msg:",pose_msg.position.z)
    print('received position:', rBW_W)
    print('received orientation:', q_WB)
    loaded_object, object_name = YCB_loader.get_ycb_object(obj_id=object_id, pos=rBW_W, quat=q_WB)

    # Initializing
    object_list = [loaded_object]
    
    # Generate the chosen Scene and Agent
    sim_factory = SimRepository.get_factory(factory_string)
    scene = sim_factory.create_scene(object_list=object_list, dt=0.0003)
    agent = sim_factory.create_robot(scene, dt=0.0003)

    agent.inhand_cam.set_cam_params(width=640, height=480)

    

    scene.start()
    

    print(f'{object_name} position')
    print(scene.get_obj_pos(obj_name = object_name))

    target_obj_id = scene.get_obj_seg_id(obj_name=object_name)
  
    print(f'for {object_name}:')
    print(scene.get_obj_seg_id(obj_name=object_name))
    # Wait time configuration
    wait_time = 0.5
    

    # Go to home position
    home_pos = [0.35, 0.0, 0.60]
    home_or = [0.0, 1.0, 0.0, 0.0]
    agent.receiveState()
    agent.robot_logger.max_time_steps = 100000000
    agent.gotoCartPositionAndQuat(home_pos, home_or, duration=12)
    print('Moved to home position')
    agent.receiveState()
    print('Current position:', agent.current_c_pos)
    print('Current orientation:', agent.current_c_quat)

    print(f'Final position of {object_name}')
    print(scene.get_obj_pos(obj_name = object_name))


    print('----------------------------')
    print('Select grasping algorithm:\n1. DEXNET \n2. GRASPNET \n3. GPD \n4. contactgraspnet \n5. Grconvnet')
    algorithm_number = input()
    if int(algorithm_number) == 1:
        print('Calling Dexnet...')
        algorithm_name = "/dexnet_bench/dexnet_grasp_planner_service"
    elif int(algorithm_number) == 2:
        print('Calling Graspnet...')
        algorithm_name = "/graspnet_bench/graspnet_grasp_planner_service"
    elif int(algorithm_number) == 3: 
        print('Calling GPD...')
        algorithm_name = "/gpd_bench/gpd_grasp_planner/gpd_grasp_planner_service"
    elif int(algorithm_number) == 4: 
        print('Calling Contactgraspnet...')
        algorithm_name = "/contact_bench/contact_grasp_planner_service"#/contact_grasp_planner_node
    else:
        print('Calling Grconvnet...')
        algorithm_name = "/grconvnet_bench/grconvnet_grasp_planner_service"#/grconvnet_grasp_planner_node

    # Calling the grasp planner
    result_from_service = call_grasp_planner(cam=agent.inhand_cam, 
                                             service_id=algorithm_name, 
                                             seg_obj_id=target_obj_id, 
                                             num_of_candidates=NUMBER_OF_CANDIDATES)
    print(result_from_service)
    grasp_pos = result_from_service[0]['position']
    grasp_quat = result_from_service[0]['quat'] 

    # Execute grasp
    agent.set_gripper_width = 0.08
    agent.wait(wait_time)
    k_grasp = quat2mat(grasp_quat)[:,2]
    target_pos = grasp_pos - 0.05*k_grasp
    agent.gotoCartPositionAndQuat(target_pos, grasp_quat, duration=12)
    agent.wait(wait_time)
    target_pos = grasp_pos - 0.005*k_grasp
    agent.gotoCartPositionAndQuat(target_pos, grasp_quat, duration=3)
    agent.receiveState()
    print('\nActual position at grasp:', agent.current_c_pos)
    print('Actual orientation at grasp:', quat2mat(agent.current_c_quat))
    agent.wait(wait_time)
    agent.close_fingers(duration=2)
    print('Fingers closed')
    agent.wait(wait_time)
    agent.gotoCartPositionAndQuat(home_pos, home_or)
    agent.wait(5)
    agent.set_gripper_width = 0.08
    agent.wait(3)

    to_real = input('Execute on real robot? (y/n)')
    if to_real == 'y':
        grasp_pose.position.x = grasp_pos[0] 
        grasp_pose.position.y = grasp_pos[1] 
        grasp_pose.position.z = grasp_pos[2]#+(0.017-0.005)  # Compensation due to table level
        grasp_pose.orientation.w = grasp_quat[0]
        grasp_pose.orientation.x = grasp_quat[1]
        grasp_pose.orientation.y = grasp_quat[2]
        grasp_pose.orientation.z = grasp_quat[3]
        for i in range(100):
            pub_grasp_pose.publish(grasp_pose)

if __name__ == "__main__":
    main()


