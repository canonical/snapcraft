# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-

import os
import snapcraft


class TgzContentHandler(snapcraft.BaseHandler):
    def __init__(self, name, options):
        super().__init__(name, options)
        self.partdir = os.path.join(os.getcwd(), "parts", self.name)
    def pull(self):
        self.run("wget -c %s " % self.options.source, cwd=self.partdir)
        tar_file = os.path.basename(self.partdir)
        return self.run("tar xf %s" % tar_file, cwd=self.partdir)
