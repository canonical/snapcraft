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
        return self.pullBranch(self.options.source) and self.run("go get -t %s" % (self.fullname))
    def build(self):
        return self.run("go build %s" % (self.fullname))
    def stage(self):
        if not self.run("go install %s" % (self.fullname)):
			return False
        return self.run("cp -vrf %s %s" % (
            os.path.join(self.godir, "bin"), self.stagedir))
    def deploy(self):
        return self.doDeploy([os.path.join(self.godir, "bin")])
    def test(self):
        return self.run("go test %s" % (self.fullname))
