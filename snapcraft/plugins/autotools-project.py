# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

import os
import snapcraft
import sys

class AutotoolsHandler(snapcraft.BaseHandler):
	def __init__(self, name, options):
		super().__init__(name, options)
		if self.options.configflags is None:
			self.options.configflags = ''
	def pull(self):
		return self.pullBranch(self.options.source)
	def build(self):
		if not os.path.exists(os.path.join(self.builddir, "configure")):
			if not self.run("env NOCONFIGURE=1 ./autogen.sh"):
				return False
		if not self.run("./configure --prefix= " + self.options.configflags):
			return False
		return self.run("make all")
	def stage(self):
		return self.run("make install DESTDIR=" + self.stagedir)
	def snap(self):
		return self.doDeploy(["bin", "share", "lib"]) # not "include"
	def test(self):
		return self.run("make check")
