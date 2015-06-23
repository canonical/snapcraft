# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

class BaseHandler:

    # The API
	def init():
		pass
	def pull():
		pass
	def build():
		pass
	def stage():
		pass
	def deploy():
		pass
	def test():
		pass

    # Helpers
	def run(cmd, dir=None):
		pass

	def pullBranch(url):
		pass
