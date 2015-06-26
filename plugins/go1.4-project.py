# -*- Mode:Python; indent-tabs-mode:t; tab-width:4 -*-

import os
import snapcraft
import sys

class Go14ProjectHandler(snapcraft.BaseHandler):
    def __init__(self, name, options):
        super().__init__(name, options)
        if self.options.source.startswith("lp:"):
            self.fullname = self.options.source.split(":~")[1]
        else:
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
        return self.run("go get -t %s" % (self.fullname), self.godir)

    def build(self):
        return self.run("go build %s" % (self.fullname), self.godir)

    def stage(self):
        if not self.run("go install %s" % (self.fullname), self.godir):
            return False
        return self.run("cp -rf %s %s" % (
            os.path.join(self.godir, "bin"), self.stagedir))

    def snap(self):
        return self.doDeploy(["bin"])

    def test(self):
        return self.run("go test %s" % (self.fullname), self.godir)
