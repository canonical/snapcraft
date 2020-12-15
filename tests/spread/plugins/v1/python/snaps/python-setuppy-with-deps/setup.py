import setuptools

# if dependencies are not handled first, processing of this
# setup.py will fail.
import six  # noqa: F401

setuptools.setup(
    name="hello-world",
    version="0.0.1",
    author="Canonical LTD",
    author_email="snapcraft@lists.snapcraft.io",
    description="A simple hello world in python",
    scripts=["hello"],
)
