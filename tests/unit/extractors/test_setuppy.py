# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from textwrap import dedent

from testscenarios import multiply_scenarios
from testtools.matchers import Equals

from snapcraft.extractors import ExtractedMetadata, _errors, setuppy
from tests import unit


class TestSetupPy:

    metadata = [
        (
            "description",
            dict(params=dict(version=None, description="test-description")),
        ),
        ("version", dict(params=dict(version="test-version", description=None))),
        (
            "key and version",
            dict(params=dict(description="test-description", version="test-version")),
        ),
    ]

    tools = [
        (
            "setuptools",
            dict(import_statement="import setuptools", method="setuptools.setup"),
        ),
        (
            "from setuptools",
            dict(import_statement="from setuptools import setup", method="setup"),
        ),
        (
            "distutils",
            dict(
                import_statement="import distutils.core", method="distutils.core.setup"
            ),
        ),
        (
            "from distutils.core",
            dict(import_statement="from distutils.core import setup", method="setup"),
        ),
    ]

    scenarios = multiply_scenarios(metadata, tools)

    def create_setuppy(self, import_statement, method, parameters):
        params = ['    {}="{}",'.format(k, v) for k, v in parameters.items() if v]

        fmt = dict(
            params="\n".join(params), import_statement=import_statement, method=method
        )

        with open("setup.py", "w") as setup_file:
            print(
                dedent(
                    """\
                {import_statement}

                {method}(
                    name='hello-world',
                {params}
                    author='Canonical LTD',
                    author_email='snapcraft@lists.snapcraft.io',
                )
            """
                ).format(**fmt),
                file=setup_file,
            )

    def test(self, tmp_work_path, import_statement, method, params):
        self.create_setuppy(import_statement, method, params)

        expected = ExtractedMetadata(**params)
        actual = setuppy.extract("setup.py", workdir=".")

        assert str(actual) == str(expected)
        assert actual == expected


class SetupPyErrorsTestCase(unit.TestCase):
    def test_unhandled_file_test_case(self):
        raised = self.assertRaises(
            _errors.UnhandledFileError, setuppy.extract, "unhandled-file", workdir="."
        )

        self.assertThat(raised.path, Equals("unhandled-file"))
        self.assertThat(raised.extractor_name, Equals("setup.py"))

    def test_bad_import(self):
        with open("setup.py", "w") as setup_file:
            print("import bad_module", file=setup_file)

        self.assertRaises(
            _errors.SetupPyImportError, setuppy.extract, "setup.py", workdir="."
        )

    def test_unsupported_setup(self):
        with open("setup.py", "w") as setup_file:
            print(
                dedent(
                    """\
                def setup(**kwargs):
                    # Raise what setuptools and distutils raise
                    raise SystemExit()

                setup(name='name', version='version')
                """
                ),
                file=setup_file,
            )

        self.assertRaises(
            _errors.SetupPyFileParseError, setuppy.extract, "setup.py", workdir="."
        )
