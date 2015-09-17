#!/usr/bin/env python3

import os
import os.path
import sys
import yaml

_CONFIG = 'configuration'

_DEFAULT_INTERVAL = 10


def main():
    config_file = os.path.join(os.environ['SNAP_APP_DATA_PATH'], _CONFIG)

    config_yaml = yaml.load(sys.stdin)
    if config_yaml:
        set_config(config_file, config_yaml)

    yaml.dump(get_config(config_file), stream=sys.stdout, default_flow_style=False)


def set_config(config_file, config_yaml={}):
    with open(config_file, 'w') as f:
        yaml.dump(_config(config_yaml), stream=f, default_flow_style=False)

    return config_yaml


def get_config(config_file):
    try:
        with open(config_file) as f:
            return yaml.load(f)
    except FileNotFoundError:
        return _config()


def _config(config_yaml={}):
    try:
        interval_value = config_yaml['config'][os.environ['SNAP_NAME']]['interval']
        if not isinstance(interval_value, int):
            config_yaml['config'][os.environ['SNAP_NAME']]['interval'] = _DEFAULT_INTERVAL
    except KeyError:
        interval = {
            'config': {
                os.environ['SNAP_NAME']: {
                    'interval': _DEFAULT_INTERVAL
                }
            }
        }
        config_yaml.update(interval)

    return config_yaml


if __name__ == '__main__':
    main()
