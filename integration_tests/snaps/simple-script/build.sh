#!/bin/bash

gcc -o test test.c

mkdir -p $1/bin
cp -a test $1/bin/
