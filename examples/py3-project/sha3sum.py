#!/usr/bin/env python3

import sys

import spongeshaker.sha3


if __name__ == '__main__':
    # 224 is the default from sha3sum
    h = spongeshaker.sha3.sha3_224()
    with open(sys.argv[1], 'rb') as fp:
        data = fp.read()
    h.update(data)
    print(h.hexdigest())
