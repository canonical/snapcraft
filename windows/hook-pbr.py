#!/usr/bin/env python
# Workaround for https://github.com/pyinstaller/pyinstaller/issues/3805 and `pbr` package
import os

os.environ["PBR_VERSION"] = "5.4.2"