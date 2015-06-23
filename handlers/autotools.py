# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

import snapcraft

class AutotoolsHandler(snapcraft.BaseHandler):
	def init():
		pass
	def pull():
		# branches to self.sourcedir by default, will be copied to self.builddir
		self.pullBranch(self.options.source)
	def build():
		self.run("env NOCONFIGURE=1 ./autogen.sh") # runs in self.builddir by default
		self.run("./configure --prefix= " + self.options.configflags)
		self.run("make all")
	def stage():
		self.run("make install DESTDIR=" + self.stagedir)
	def deploy():
		return ["bin", "share", **self.findFiles("lib/**/*.so")] # not "include"
	def test():
		self.run("make check")
