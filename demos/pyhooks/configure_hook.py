#!/usr/bin/env python3

import subprocess
import sys


def main():
    output = subprocess.check_output(["snapctl", "get", "fail"]).decode("utf8").strip()
    if output == "true":
        print("Failing as requested.")
        sys.exit(1)

    print("I'm the configure hook!")


if __name__ == "__main__":
    main()
