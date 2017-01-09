import os
import shutil

import subprocess

import time

from SwarmBootstrapUtils import yaml_parser


def point_camera_downward(my_env, tracker, log_dir):
    point_camera_cmd = 'rostopic pub /bebop/camera_control geometry_msgs/Twist [0.0,0.0,' \
                       '0.0] [0.0,-50.0,0.0]'
    execute_cmd(point_camera_cmd, my_env, log_dir + '/point_cam_downward.log', tracker)


def launch_bebop_autonomy(bebop_ip, my_env, tracker, log_dir):
    bebop_launch_cmd = 'roslaunch bebop_driver bebop_node.launch ip:=' + bebop_ip

    count = 1
    while True:
        bebop_autonomy = execute_cmd(bebop_launch_cmd, my_env,
                                     log_dir + '/bebop_autonomy' + str(count) + '.log', tracker)
        count += 1
        listen_to_odom_cmd = 'rostopic echo -n 2 /bebop/image_raw'
        listen_process = subprocess.Popen(listen_to_odom_cmd.split(), env=my_env,
                                          stdout=subprocess.DEVNULL)
        time.sleep(10)
        status = listen_process.poll()
        if status is None:
            print('Did not receive any image. Relaunch bebop_autonomy.')
            execute_cmd_and_wait('rosnode kill /bebop/bebop_driver', my_env,
                                 log_dir + '/cleanup' + str(count) + '.log')
            listen_process.kill()
            while bebop_autonomy.poll() is None and listen_process.poll() is None:
                time.sleep(0.1)
                rosnode_cleanup = subprocess.Popen('rosnode cleanup'.split(), env=my_env,
                                                   stdin=subprocess.PIPE)
                rosnode_cleanup.communicate(b'y\n')
                rosnode_cleanup.wait()

            time.sleep(1)
        else:
            print('Received 2 image_raw messages')
            return


def launch_arlocros(my_env, number_of_arlocros, tracker, log_dir):
    rats_dir = execute_cmd_and_get_output('rospack find rats')
    # delete build script folder
    build_script_dir = rats_dir + '/ARLocROS/build/scripts'
    shutil.rmtree(build_script_dir, ignore_errors=True)
    # set jvm use 3g of heaps
    arlocros_run_script_path = rats_dir + '/ARLocROS/build/install/ARLocROS/bin/ARLocROS'
    with open(arlocros_run_script_path, 'r') as original_file:
        content = original_file.read()

    if 'DEFAULT_JVM_OPTS=""' in content:
        content = content.replace('DEFAULT_JVM_OPTS=""', 'DEFAULT_JVM_OPTS="-Xmx3g -Xms3g"')
        with open(arlocros_run_script_path, 'w') as new_file:
            new_file.write(content)

    for i in range(number_of_arlocros):
        # FIXME make this ros param explicit in the configuration
        set_param_cmd = 'rosparam set ' + '/ARLocROS' + str(i) + '/instance_id ' + str(i)
        execute_cmd_and_wait(set_param_cmd, my_env, log_dir + '/launch_arlocros' + str(i) + '.log')
        # launch the java node
        arlocros_launch_cmd = 'rosrun rats ARLocROS arlocros.ARLoc __name:=ARLocROS' + str(i)
        execute_cmd(arlocros_launch_cmd, my_env, log_dir + '/launch_arlocros' + str(i) + '.log',
                    tracker)


def record_rosbag(my_env, tracker, log_dir):
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    record_rosbag_cmd = 'rosbag record -o ' + log_dir + '/ /bebop/image_raw /bebop/cmd_vel ' \
                                                        '/bebop/odom /tf ' \
                                                        '/bebop/camera_info /arlocros/marker_pose' \
                                                        ' /arlocros/fused_pose'
    execute_cmd(record_rosbag_cmd, my_env, log_dir + '/record_rosbag.log', tracker)


def launch_xbox_controller(my_env, tracker, log_dir):
    launch_xbox_controller_cmd = 'roslaunch bebop_tools joy_teleop.launch joy_config:=xbox360'
    execute_cmd(launch_xbox_controller_cmd, my_env, log_dir + '/launch_xbox.log', tracker)


def launch_beswarm(my_env, tracker, beswarm_config, log_dir):
    my_env['LOG_DIR'] = log_dir  # delete build script folder
    build_script_dir = execute_cmd_and_get_output('rospack find rats') + '/BeSwarm/build/scripts'
    shutil.rmtree(build_script_dir, ignore_errors=True)
    # launch the java node
    beswarm_launch_cmd = 'rosrun rats BeSwarm ' + beswarm_config['javanode'] + ' __name:=' + \
                         beswarm_config['nodename']
    execute_cmd(beswarm_launch_cmd, my_env, log_dir + '/launch_beswarm.log', tracker)


def start_synchronizer(synchronizer_config, tracker, log_dir, config_dir):
    my_env = os.environ.copy()
    my_env['ROS_MASTER_URI'] = 'http://localhost:' + synchronizer_config['ros_master_port']
    launch_ros_master(my_env, synchronizer_config['ros_master_port'],
                      synchronizer_config['sync_config'], tracker, config_dir, log_dir)
    set_ros_parameters(my_env, synchronizer_config['rosparam'], log_dir)
    synchronizer_launch_cmd = 'rosrun rats ' + synchronizer_config['python_node']
    execute_cmd(synchronizer_launch_cmd, my_env, log_dir + '/launch_synchronizer.log', tracker)


def start_pose_aggregation(pose_aggregation_config, tracker, log_dir, config_dir):
    my_env = os.environ.copy()
    my_env['ROS_MASTER_URI'] = 'http://localhost:' + pose_aggregation_config['ros_master_port']
    launch_ros_master(my_env, pose_aggregation_config['ros_master_port'],
                      pose_aggregation_config['sync_config'], tracker, config_dir, log_dir)
    rosbag_cmd = 'rosbag record -a -o ' + log_dir + '/'
    execute_cmd(rosbag_cmd, my_env, log_dir + '/record_rosbag.log', tracker)


def launch_ros_master(my_env, port, sync_config_file, tracker, config_dir, log_dir):
    master_sync_config_file = config_dir + '/' + sync_config_file
    if os.path.isfile(master_sync_config_file):
        # start a ros master
        roscore_cmd = 'roscore -p ' + port
        execute_cmd(roscore_cmd, my_env, log_dir + '/roscore.log', tracker)
        time.sleep(2)
        # start master_discovery_fkie (to discover other ros masters)
        master_discovery_cmd = 'rosrun master_discovery_fkie master_discovery ' \
                               '_mcast_group:=224.0.0.1'
        execute_cmd(master_discovery_cmd, my_env, log_dir + '/master_discovery.log', tracker)
        # start master_sync_fkie (to sync with other ros masters
        substituted_master_sync_config_file = yaml_parser.substitute(master_sync_config_file)
        sync_cmd = 'rosrun master_sync_fkie master_sync _interface_url:=' + \
                   substituted_master_sync_config_file
        execute_cmd(sync_cmd, my_env, log_dir + '/sync_cmd.log', tracker)
    else:
        print('FILE NOT FOUND: ', master_sync_config_file)
        exit()


def load_ros_parameters_from_file(file_path, my_env, log_dir):
    if os.path.isfile(file_path):
        substituted_file = yaml_parser.substitute(file_path)
        load_param_cmd = 'rosparam load ' + substituted_file
        execute_cmd_and_wait(load_param_cmd, my_env, log_dir + '/rosparam_load.log')
    else:
        print('FILE NOT FOUND: ', file_path)
        exit()


def set_ros_parameters(my_env, ros_params, log_dir):
    log_file = log_dir + '/set_rosparam.log'
    for key, value in ros_params.items():
        set_param_cmd = 'rosparam set ' + str(key) + ' ' + str(value)
        execute_cmd_and_wait(set_param_cmd, my_env, log_file)


def relay_topics(my_env, topics, namespace, tracker, log_dir):
    for topic in topics:
        relay_cmd = 'rosrun topic_tools relay ' + topic + ' /' + namespace + topic
        execute_cmd(relay_cmd, my_env, log_dir + '/topic_relay.log', tracker)


def relay_one_topic(my_env, input_topic, output_topic, tracker, logdir):
    relay_cmd = 'rosrun topic_tools relay ' + input_topic + ' ' + output_topic
    execute_cmd(relay_cmd, my_env, logdir + '/one_topic_relay.log', tracker)


def execute_cmd(cmd, my_env, log_file_abs_path, tracker):
    print(cmd)
    log_file = open_file(log_file_abs_path)
    process = subprocess.Popen(cmd.split(), env=my_env, stdout=log_file, stderr=subprocess.STDOUT)
    tracker['processes'].append(process)
    tracker['opened_files'].append(log_file)
    return process


def execute_cmd_and_wait(cmd, my_env, log_file_abs_path):
    print(cmd)
    log_file = open_file(log_file_abs_path)
    subprocess.call(cmd.split(), env=my_env, stdout=log_file, stderr=subprocess.STDOUT)
    log_file.close()


def execute_cmd_and_get_output(cmd):
    print(cmd)
    return subprocess.check_output(cmd.split()).decode("utf-8").rstrip()


def open_file(file_abs_path):
    os.makedirs(os.path.dirname(file_abs_path), exist_ok=True)
    log_file = open(file_abs_path, 'a+')
    return log_file
