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
        self.goroot = os.path.join(os.path.join(
                os.getcwd(), "parts", "go1.4", "go"))
        self.gorootbin = os.path.join(self.goroot, "bin")
        self.tar_file = os.path.join(
			self.godir, os.path.basename(URLS[self.arch]))
        try:
            os.makedirs(self.godir)
        except FileExistsError:
            pass
    def env(self):
        return [
            "PATH=%s:$PATH" % self.gorootbin,
            "GOROOT=%s" % self.goroot,
        ]
    def pull(self):
		# FIXME: use the internal downloader (once its there) to get stuff
        if not self.run("wget -c %s " % URLS[self.arch], cwd=self.godir):
			return False
        if not os.path.exists(os.path.join(self.godir, "go/bin/go")):
            return self.run("tar xf %s" % self.tar_file, cwd=self.godir)
		return True
