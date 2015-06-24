# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

import os
import subprocess
import sys

class BaseHandler:

	def __init__(self, name, options):
		self.name = name
		self.options = options
		self.sourcedir = os.path.join(os.getcwd(), "parts", self.name, "src")
		self.builddir = os.path.join(os.getcwd(), "parts", self.name, "build")
		self.stagedir = os.path.join(os.getcwd(), "parts", "stage")
		self.snapdir = os.path.join(os.getcwd(), "snap")

    # The API
	def init(self):
		try: os.makedirs(self.sourcedir)
		except: pass
		try: os.makedirs(self.builddir)
		except: pass
		try: os.makedirs(self.stagedir)
		except: pass
	def pull(self):
		pass
	def build(self):
		self.run("cp -Trf " + self.sourcedir + " " + self.builddir)
	def stage(self):
		pass
	def deploy(self):
		pass
	def test(self):
		pass

    # Helpers
	def run(self, cmd, cwd=None):
		if cwd is None:
			cwd = self.builddir
		if False:
			print(cmd)
		subprocess.call(cmd, shell=True, cwd=cwd)

	def pullBranch(self, url):
		if url.startswith("bzr:") or url.startswith("lp:"):
			if os.path.exists(os.path.join(self.sourcedir, ".bzr")):
				self.run("bzr pull", self.sourcedir)
			else:
				self.run("bzr branch " + url + " .", self.sourcedir)
		elif url.startswith("git:"):
			if os.path.exists(os.path.join(self.sourcedir, ".git")):
				self.run("git pull", self.sourcedir)
			else:
				self.run("git clone " + url + " .", self.sourcedir)
		else:
			raise Exception("Did not recognize branch url: " + url)

	def doDeploy(self, dirs):
		try: os.makedirs(self.snapdir)
		except: pass

		for d in dirs:
			self.run("cp -vr " + d + " " + self.snapdir, cwd=self.stagedir)

