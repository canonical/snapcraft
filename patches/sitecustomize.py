import site
import os

snap_dir = os.getenv("SNAP")
snapcraft_stage = os.getenv("SNAPCRAFT_STAGE")
snapcraft_part_install = os.getenv("SNAPCRAFT_PART_INSTALL")


def add_site_pkg(root: str) -> None:
    site.addsitedir(os.path.join(root, "lib/python3.8/site-packages"))


if snapcraft_part_install:
    add_site_pkg(snapcraft_part_install)
elif snap_dir:
    add_site_pkg(snap_dir)
    site.ENABLE_USER_SITE = False
