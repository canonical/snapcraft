# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

import importlib.machinery
import os
import snapcraft
import subprocess
import sys
import yaml


class Plugin:

	def __init__(self, pluginDir, name, partName, properties, optionsOverride=None, loadCode=True, loadConfig=True):
		self.valid = False
		self.code = None
		self.config = None
		self.partNames = []

		self.sourcedir = os.path.join(os.getcwd(), "parts", partName, "src")
		self.builddir = os.path.join(os.getcwd(), "parts", partName, "build")
		self.stagedir = os.path.join(os.getcwd(), "stage")
		self.snapdir = os.path.join(os.getcwd(), "snap")
		self.statefile = os.path.join(os.getcwd(), "parts", partName, "state")

		if loadConfig:
			configPath = os.path.join(pluginDir, name + ".yaml")
			if not os.path.exists(configPath):
				snapcraft.common.log("Missing config for part %s" % (name), file=sys.stderr)
				return
			self.config = yaml.load(open(configPath, 'r')) or {}

		codePath = os.path.join(pluginDir, name + ".py")
		if loadCode and os.path.exists(codePath):
			class Options(): pass
			options = Options()

			if self.config:
				for opt in self.config.get('options', []):
					if opt in properties:
						setattr(options, opt, properties[opt])
					else:
						if self.config['options'][opt].get('required', False):
							snapcraft.common.log("Required field %s missing on part %s" % (opt, name), file=sys.stderr)
							return
						setattr(options, opt, None)
			if optionsOverride:
				options = optionsOverride

			loader = importlib.machinery.SourceFileLoader("snapcraft.plugins." + name, codePath)
			module = loader.load_module()
			for propName in dir(module):
				prop = getattr(module, propName)
				if issubclass(prop, snapcraft.BaseHandler):
					self.code = prop(partName, options)
					break

		self.partNames.append(partName)
		self.valid = True

	def makedirs(self):
		try: os.makedirs(self.sourcedir)
		except: pass
		try: os.makedirs(self.builddir)
		except: pass
		try: os.makedirs(self.stagedir)
		except: pass
		try: os.makedirs(self.snapdir)
		except: pass

	def isValid(self):
		return self.valid

	def names(self):
		return self.partNames

	def notifyStage(self, stage, hint=''):
		snapcraft.common.log(stage + " " + self.partNames[0] + hint)

	def isDirty(self, stage):
		try:
			with open(self.statefile, 'r') as f:
				lastStep = f.read()
				return snapcraft.common.commandOrder.index(stage) > snapcraft.common.commandOrder.index(lastStep)
		except Exception:
			return True

	def shouldStageRun(self, stage, force):
		if not force and not self.isDirty(stage):
			self.notifyStage('Skipping ' + stage, ' (already ran)')
			return False
		return True

	def markDone(self, stage):
		with open(self.statefile, 'w+') as f:
			f.write(stage)

	def pull(self, force=False):
		if not self.shouldStageRun('pull', force): return True
		self.makedirs()
		if self.code and hasattr(self.code, 'pull'):
			self.notifyStage("Pulling")
			if not getattr(self.code, 'pull')():
				return False
			self.markDone('pull')
		return True

	def build(self, force=False):
		if not self.shouldStageRun('build', force): return True
		self.makedirs()
		subprocess.call(['cp', '-Trf', self.sourcedir, self.builddir])
		if self.code and hasattr(self.code, 'build'):
			self.notifyStage("Building")
			if not getattr(self.code, 'build')():
				return False
			self.markDone('build')
		return True

	def test(self, force=False):
		if not self.shouldStageRun('test', force): return True
		self.makedirs()
		if self.code and hasattr(self.code, 'test'):
			self.notifyStage("Testing")
			if not getattr(self.code, 'test')():
				return False
			self.markDone('test')
		return True

	def stage(self, force=False):
		if not self.shouldStageRun('stage', force): return True
		self.makedirs()
		if self.code and hasattr(self.code, 'stage'):
			self.notifyStage("Staging")
			if not getattr(self.code, 'stage')():
				return False
			self.markDone('stage')
		return True

	def snap(self, force=False):
		if not self.shouldStageRun('snap', force): return True
		self.makedirs()
		if self.code and hasattr(self.code, 'snap'):
			self.notifyStage("Deploying")
			if not getattr(self.code, 'snap')():
				return False
			self.markDone('snap')
		return True

	def env(self):
		if self.code and hasattr(self.code, 'env'):
			return getattr(self.code, 'env')()
		return []
