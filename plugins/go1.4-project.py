# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

import os
import snapcraft
import sys

class Go14ProjectHandler(snapcraft.BaseHandler):
    def __init__(self, name, options):
        super().__init__(name, options)
        self.fullname = self.options.source.split("://")[1]
        self.goroot = os.path.join(os.path.join(
                os.getcwd(), "parts", "go1.4", "go"))
        self.gorootbin = os.path.join(self.goroot, "bin")
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
        self.run("PATH=%s:$PATH GOROOT=%s GOPATH=%s go get -v %s" % (
			self.gorootbin, self.goroot, self.godir, self.fullname))
    def build(self):
        super().build()
        self.run("PATH=%s:$PATH GOROOT=%s GOPATH=%s go build %s" % (
			self.gorootbin, self.goroot, self.godir, self.fullname))
    def stage(self):
        super().stage()
        self.run("PATH=%s:$PATH GOROOT=%s GOPATH=%s go install %s" % (self.gorootbin, self.goroot, self.godir, self.fullname))
    def deploy(self):
        super().deploy()
        self.doDeploy([os.path.join(self.godir, "bin")])
    def test(self):
        super().test()
        self.run("PATH=%s:$PATH GOROOT=%s GOPATH=%s go test %s" % (self.gorootbin, self.goroot, self.godir, self.fullname))


if __name__ == "__main__":
    class Options:
        source = "git://github.com/mvo5/godd"
        configflags = ""
    handler = Go14ProjectHandler("godd", Options())
    getattr(handler, sys.argv[1])()
