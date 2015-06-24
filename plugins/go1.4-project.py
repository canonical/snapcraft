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
    def init(self):
        super().init()
    def pull(self):
        super().pull()
        self.pullBranch(self.options.source)
        self.run("GOPATH=%s go get -v %s" % (self.godir, self.fullname))
    def build(self):
        super().build()
        self.run("GOPATH=%s go build %s" % (self.godir, self.fullname))
    def stage(self):
        super().stage()
        self.run("GOPATH=%s go install %s" % (self.godir, self.fullname))
    def deploy(self):
        super().deploy()
        self.doDeploy([os.path.join(self.godir, "bin")])
    def test(self):
        super().test()
        self.run("GOPATH=%s go test %s" % (self.godir, self.fullname))


if __name__ == "__main__":
	class Options:
		source = "git://github.com/mvo5/godd"
		configflags = ""
	handler = Go14ProjectHandler("godd", Options())
	getattr(handler, sys.argv[1])()
