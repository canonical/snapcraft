# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

import os
import snapcraft
import sys

class Go14ProjectHandler(snapcraft.BaseHandler):
    def __init__(self, name, options):
        super().__init__(name, options)
        self.fullname = self.options.source.split("://")[1]
        self.godir = os.path.join(os.path.join(os.getcwd(), "parts", self.name))
        try:
            os.makedirs(self.godir)
        except FileExistsError:
            pass
    def env(self):
        return [
            "GOPATH=%s" % self.godir,
        ]
    def pull(self):
        self.pullBranch(self.options.source)
        self.run("go get -t %s" % (self.fullname))
    def build(self):
        self.run("go build %s" % (self.fullname))
    def stage(self):
        self.run("go install %s" % (self.fullname))
    def deploy(self):
        self.doDeploy([os.path.join(self.godir, "bin")])
    def test(self):
        self.run("go test %s" % (self.fullname))


if __name__ == "__main__":
    class Options:
        source = "git://github.com/mvo5/godd"
        configflags = ""
    handler = Go14ProjectHandler("godd", Options())
    getattr(handler, sys.argv[1])()
