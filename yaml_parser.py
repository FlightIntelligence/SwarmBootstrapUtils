import os

import yaml
import uuid


def read(yaml_file):
    """
    Reads a yaml file and translate it to a python dictionary.
    :param yaml_file: the absolute directory of the yaml file to be read
    :type yaml_file: str
    :return: the python dictionary representing the content of the yaml file
    :rtype: dict
    """
    with open(yaml_file, 'r') as stream:
        try:
            return yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)


def substitute(yaml_file, postfix='_tmp'):
    """
    Parses a yaml file. A new temporary yaml file will be generated based on the input file with
    all the substitution arguments replaced by their values.
    :param postfix: the postfix of the generated file
    :type postfix: str
    :param yaml_file: the absolute directory of the yaml file to be parsed
    :type yaml_file: str
    :return: the absolute directory of the new temporary yaml file
    :rtype: str
    """
    file = open(yaml_file, 'r')
    content = file.read()
    file.close()

    config_dir = os.path.dirname(yaml_file)
    # replace the absolute path variables
    content = content.replace('${config_dir}', config_dir)

    # read all other variables
    variables = read(config_dir + '/variables.yaml')

    # replace all substitution arguments/variables by their values defined in variables.yaml
    for key, value in variables.items():
        content = content.replace('${' + key + '}', value)

    # check if there is any substitution argument not being defined in variables.yaml
    index = content.find('${')
    if index != -1:
        print('Cannot parse ' + yaml_file + '. Variable ' + content[index:content.find(
            '}') + 1] + ' is not defined in variables.yaml.')
        # if such variable exists, exit immediately
        exit()

    # the absolute directory of the temporary file
    unique_id = uuid.uuid4()
    substituted_file = yaml_file.replace('.yaml', postfix + unique_id + '.yaml')

    # write the temporary file to disk
    tmp_file = open(substituted_file, 'w')
    tmp_file.write(content)
    tmp_file.close()

    # return the absolute directory of the temporary file
    return substituted_file
