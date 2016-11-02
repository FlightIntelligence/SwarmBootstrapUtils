import os
import sys
from distutils import dir_util

from SwarmBootstrapUtils import executor
from SwarmBootstrapUtils import yaml_parser


def log_commit_numbers(log_dir):
    log_info = ''
    log_info += executor.execute_cmd_and_get_output('git rev-parse --show-toplevel HEAD')
    log_info += '\n'
    log_info += executor.execute_cmd_and_get_output(
        'git submodule foreach -q git rev-parse --show-toplevel HEAD')

    log_file = log_dir + '/commit_numbers.log'
    os.makedirs(os.path.dirname(log_file), exist_ok=True)
    with open(log_file, 'a+') as f:
        f.write(log_info)


def copy_config_to_log_dir(config_dir, log_dir):
    log_config_dir = log_dir + '/config/' + config_dir.split('/')[-1]
    dir_util.copy_tree(config_dir, log_config_dir)
    return log_config_dir


def check_unique_integer_id(bebop_configs):
    ids = {}
    for bebop, config in bebop_configs.items():
        unique_integer_id = config['beswarm_config']['rosparam']['unique_integer_id']
        if unique_integer_id in ids:
            print(str(bebop) + ' has the same integer id number ' + str(
                unique_integer_id) + ' with ' + str(ids[unique_integer_id]))
            exit()
        else:
            ids[unique_integer_id] = bebop


def get_main_config(config_dir):
    config_file = config_dir + '/config.yaml'

    if os.path.isfile(config_file):
        # parse the main config file
        substituted_config_file = yaml_parser.substitute(config_file)
        # convert the substituted config file to python dictionary
        configs = yaml_parser.read(substituted_config_file)
        return configs
    else:
        print('FILE NOT FOUND: ', config_file)
        exit()


def get_config_dir():
    try:
        config_dir = sys.argv[1]
    except IndexError:
        print('Please pass the absolute path of the configuration folder')
        exit()

    if config_dir[-1] == '/':
        config_dir = config_dir[:-1]

    if os.path.isdir(config_dir):
        return config_dir
    else:
        print(config_dir, ' is not a valid directory')
        exit()
