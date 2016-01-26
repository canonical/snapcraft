#!/usr/bin/env python3

import argparse

from paho.mqtt import client as mqtt_client


DEFAULT_HOST = 'localhost'
DEFAULT_PORT = 1883

topic = ''


def on_connect(client, userdata, unused1, unused2):
    # Ignore the unused arguments.
    del unused1, unused2
    print('MQTT subscriber connected.')
    client.subscribe(topic)


def on_message(unused1, unused2, message):
    # Ignore the unused arguments.
    del unused1, unused2
    print(message.topic + ' ' + str(message.payload))


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
