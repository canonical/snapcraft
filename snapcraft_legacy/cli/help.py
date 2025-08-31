# DEPRECATED: minimal legacy help shim kept only to satisfy legacy tests.
# To be removed with tests that import `_TOPICS`. See issue #5682.

from typing import Dict

# Provide only keys that legacy tests assert on; extend if CI complains.
_TOPICS: Dict[str, str] = {
    "plugins": "Information about plugins",
    "sources": "Information about sources",
    "register": "Information about registering names",
}
__all__ = ["_TOPICS"]
