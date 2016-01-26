#!/usr/bin/env python3

import argparse

from paho.mqtt import client as mqtt_client


DEFAULT_HOST = 'localhost'
DEFAULT_PORT = 1883


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
    parser.add_argument('topic', help='The topic to publish to.')
    parser.add_argument('payload', help='The payload to send to the topic.')
    args = parser.parse_args()

    client = mqtt_client.Client()
    client.connect(args.host, args.port)

    try:
        client.loop_start()
        client.publish(args.topic, args.payload)
    finally:
        client.loop_stop()


if __name__ == "__main__":
    main()
