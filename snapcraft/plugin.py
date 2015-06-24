# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

import importlib.machinery
import os
import snapcraft
import sys
import yaml


class Plugin:
	code = None
	config = None

	def __init__(self, pluginDir, name, properties):
		configPath = os.path.join(pluginDir, name + ".yaml")
		self.config = yaml.load(open(configPath, 'r'))

		codePath = os.path.join(pluginDir, name + ".py")
		if os.path.exists(codePath):
			class Options(): pass
			options = Options()

			for opt in self.config.get('options', []):
				if opt in properties:
					setattr(options, opt, properties[opt])
				else:
					if self.config['options'][opt].get('required', False):
						self.config = None
						print("Required field %s missing on part %s" % (opt, name), file=sys.stderr)
						return
					setattr(options, opt, None)

			loader = importlib.machinery.SourceFileLoader("snapcraft.plugins." + name, codePath)
			module = loader.load_module()
			for propName in dir(module):
				prop = getattr(module, propName)
				if issubclass(prop, snapcraft.BaseHandler):
					self.code = prop(name, options)
					break


	def isValid(self):
		return self.code or self.config

	def init(self):
		if self.code and hasattr(self.code, 'init'):
			return getattr(self.code, 'init')()

	def pull(self):
		if self.code and hasattr(self.code, 'pull'):
			return getattr(self.code, 'pull')()

	def build(self):
		if self.code and hasattr(self.code, 'build'):
			return getattr(self.code, 'build')()

	def test(self):
		if self.code and hasattr(self.code, 'test'):
			return getattr(self.code, 'test')()

	def stage(self):
		if self.code and hasattr(self.code, 'stage'):
			return getattr(self.code, 'stage')()

	def deploy(self):
		if self.code and hasattr(self.code, 'deploy'):
			return getattr(self.code, 'deploy')()
