import os
import sys

from SwarmBootstrapUtils import yaml_parser


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
