# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

import os
import snapcraft
import sys

# FIXME: add support for i386
URLS = {
        'amd64': "https://storage.googleapis.com/golang/go1.4.2.linux-amd64.tar.gz",
}

class Go14Handler(snapcraft.BaseHandler):
    def __init__(self, name, options):
        super().__init__(name, options)
        # FIXME: horrible
        self.arch = "amd64"
        self.godir = os.path.join(
			os.path.join(os.getcwd(), "parts", self.name))
        self.tar_file = os.path.join(
			self.godir, os.path.basename(URLS[self.arch]))
        try:
            os.makedirs(self.godir)
        except FileExistsError:
            pass
    def init(self):
        super().init()
    def pull(self):
		# FIXME: use the internal downloader (once its there) to get stuff
        self.run("wget -c %s " % URLS[self.arch], cwd=self.godir)
    def build(self):
        super().build()
        if not os.path.exists(os.path.join(self.godir, "go/bin/go")):
            self.run("tar xf %s" % self.tar_file, cwd=self.godir)
    def stage(self):
        pass
    def deploy(self):
        pass
    def test(self):
        pass


if __name__ == "__main__":
    class Options:
        pass
    handler = Go14Handler("go1.4", Options())
    getattr(handler, sys.argv[1])()
