# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

import os
import snapcraft
import sys


class TgzContentHandler(snapcraft.BaseHandler):
    def __init__(self, name, options):
        super().__init__(name, options)
        self.partdir = os.path.join(os.getcwd(), "parts", self.name)
    def init(self):
        super().init()
    def pull(self):
        self.run("wget -c %s " % self.options.source, cwd=self.partdir)
        tar_file = os.path.basename(self.partdir)
        self.run("tar xf %s" % tar_file, cwd=self.partdir)
    def build(self):
        super().build()
    def stage(self):
        super().stage()
    def deploy(self):
        super().deploy()
    def test(self):
        super().test()


