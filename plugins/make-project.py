# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

import snapcraft

class MakeHandler(snapcraft.BaseHandler):
	def pull(self):
		self.pullBranch(self.options.source)
	def build(self):
		self.run("make all")
	def stage(self):
		self.run("make install DESTDIR=" + self.stagedir)
	def deploy(self):
		self.doDeploy(["bin", "share", "lib"]) # not "include"
	def test(self):
		self.run("make check")
