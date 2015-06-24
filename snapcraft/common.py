# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

# Data/methods shared between plugins and snapcraft

import os
import subprocess
import tempfile

env = []

def assembleEnv():
	return '\n'.join(['export ' + e for e in env])

def run(cmd, **kwargs):
	# FIXME: This is gross to keep writing this, even when env is the same
	if isinstance(cmd, list):
		cmd = ' '.join(cmd)
	with tempfile.NamedTemporaryFile(mode='w+') as f:
		f.write(assembleEnv(env))
		f.write('\n')
		f.write('exec ' + cmd)
		f.flush()
		return subprocess.call(['/bin/sh', f.name], **kwargs) == 0

def log(msg, file=None):
	print('\033[01m' + msg + '\033[0m', file=None)

commandOrder = ["pull", "build", "test", "stage", "deploy"]
stagedir = os.path.join(os.getcwd(), "staging")
snapdir = os.path.join(os.getcwd(), "snap")
