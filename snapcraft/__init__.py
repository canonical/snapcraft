# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

import os
import snapcraft.common
import subprocess
import sys

class BaseHandler:

	def __init__(self, name, options):
		self.name = name
		self.options = options
		self.sourcedir = os.path.join(os.getcwd(), "parts", self.name, "src")
		self.builddir = os.path.join(os.getcwd(), "parts", self.name, "build")
		self.stagedir = os.path.join(os.getcwd(), "staging")
		self.snapdir = os.path.join(os.getcwd(), "snap")

    # The API
	def pull(self):
		return True
	def build(self):
		return True
	def stage(self):
		return True
	def deploy(self):
		return True
	def test(self):
		return True
	def env(self):
		return []

    # Helpers
	def run(self, cmd, cwd=None, **kwargs):
		if cwd is None:
			cwd = self.builddir
		if True:
			print(cmd)
		return snapcraft.common.run(cmd, cwd=cwd, **kwargs)

	def pullBranch(self, url):
		if url.startswith("bzr:") or url.startswith("lp:"):
			if os.path.exists(os.path.join(self.sourcedir, ".bzr")):
				return self.run("bzr pull " + url, self.sourcedir)
			else:
				os.rmdir(self.sourcedir)
				return self.run("bzr branch " + url + " " + self.sourcedir)
		elif url.startswith("git:"):
			if os.path.exists(os.path.join(self.sourcedir, ".git")):
				return self.run("git pull", self.sourcedir)
			else:
				return self.run("git clone " + url + " .", self.sourcedir)
		else:
			raise Exception("Did not recognize branch url: " + url)

	def doDeploy(self, dirs):
		try: os.makedirs(self.snapdir)
		except: pass

		for d in dirs:
			if os.path.exists(os.path.join(self.stagedir,d)):
				try: os.makedirs(os.path.join(self.snapdir, d))
				except: pass
				if not self.run("cp -vrf " + d + " " + self.snapdir + "/", cwd=self.stagedir):
					return False
		return True

