# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

# Data/methods shared between plugins and snapcraft

import subprocess
import tempfile

env = []
def run(cmd, cwd=None):
	# FIXME: This is gross to keep writing this, even when env is the same
	with tempfile.NamedTemporaryFile(mode='w+') as f:
		f.write('export ')
		f.write('\nexport '.join(env))
		f.write('\n')
		f.write('exec ' + cmd)
		f.flush()
		return subprocess.call(['/bin/sh', f.name], cwd=cwd) == 0

def log(msg):
	print('\033[01m' + msg + '\033[0m')

commandOrder = ["pull", "build", "test", "stage", "deploy"]
