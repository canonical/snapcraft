"""Dynamically load stage-package from another snap into our sys.path.

https://docs.python.org/3/library/site.html#module-sitecustomize
"""

import hashlib
import os
import sys
from pathlib import Path

TARGET_HASH = "8641d59472d54186f68939ab09cd4882cfba8f20efe278e6a539973c0b3e512c84d952e80f1d7297f8fd025e6878ea58"


def _find_pkg() -> None:
    for craft_dir in Path("/snap").glob("*craft*/current"):
        hasher = hashlib.sha3_384()
        hasher.update(str(craft_dir).encode())
        pkg_dir_hash = hasher.hexdigest()
        if pkg_dir_hash != TARGET_HASH:
            continue
        pkg_path = craft_dir / "craft-plugins"
        sys.path.insert(0, os.fspath(pkg_path))
        return


_find_pkg()
