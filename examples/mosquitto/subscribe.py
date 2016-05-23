#!/usr/bin/env python3

import argparse
import os
import sys

from paho.mqtt import client as mqtt_client


DEFAULT_HOST = 'localhost'
DEFAULT_PORT = 1883

topic = ''


def on_connect(client, userdata, unused1, unused2):
    # Ignore the unused arguments.
    del unused1, unused2
    _log('MQTT subscriber connected.')
    client.subscribe(topic)


def on_message(unused1, unused2, message):
    # Ignore the unused arguments.
    del unused1, unused2
    _log(message.topic + ' ' + str(message.payload))
    # XXX Exit on first message simplifyies the tests a lot, so this
    # subscriber can get only one message. --elopio - 2016-05-02
    sys.exit(0)


def _log(message):
    print(message)
    log_file_path = os.path.join(
        os.getenv('SNAP_USER_DATA'), 'mosquitto.subscriber.log')
    with open(log_file_path, 'a') as log_file:
        log_file.write(message + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'host', nargs='?', default=DEFAULT_HOST,
        help=('The IP or hostname of the MQTT server. '
              'Defaults to {}.'.format(DEFAULT_HOST)))
    parser.add_argument(
        'port', type=int, nargs='?', default=DEFAULT_PORT,
        help=('The port of the MQTT server. '
              'Defaults to {}.'.format(DEFAULT_PORT)))
    parser.add_argument('topic', help='The topic to subscribe to.')
    args = parser.parse_args()

    client = mqtt_client.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    global topic
    topic = args.topic
    client.connect(args.host, args.port)
    client.loop_forever()


if __name__ == "__main__":
    main()
