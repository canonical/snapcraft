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

from snapcraft.extractors import setuppy, ExtractedMetadata

from testscenarios import multiply_scenarios
from testtools.matchers import Equals

from snapcraft.extractors import _errors
from tests import unit


class SetupPyTestCase(unit.TestCase):

    metadata = [
        (
            "summary",
            {
                "params": [
                    {
                        "key": "description",
                        "param_name": "summary",
                        "value": "test-summary",
                        "expect": "test-summary",
                    }
                ]
            },
        ),
        (
            "description",
            {
                "params": [
                    {
                        "key": "long_description",
                        "param_name": "description",
                        "value": "test-description",
                        "expect": "test-description",
                    }
                ]
            },
        ),
        (
            "version",
            {
                "params": [
                    {
                        "key": "version",
                        "param_name": "version",
                        "value": "test-version",
                        "expect": "test-version",
                    }
                ]
            },
        ),
        (
            "summary, description and version",
            {
                "params": [
                    {
                        "key": "description",
                        "param_name": "summary",
                        "value": "test-summary",
                        "expect": "test-summary",
                    },
                    {
                        "key": "long_description",
                        "param_name": "description",
                        "value": "test-description",
                        "expect": "test-description",
                    },
                    {
                        "key": "version",
                        "param_name": "version",
                        "value": "test-version",
                        "expect": "test-version",
                    },
                ]
            },
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

    def setUp(self):
        super().setUp()

        params = [
            '    {}="{}",'.format(p["key"], p["value"])
            for p in self.params
            if p["value"]
        ]
        print("params:::" + str(params))
        fmt = dict(
            params="\n".join(params),
            import_statement=self.import_statement,
            method=self.method,
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

    def test_info_extraction(self):
        kwargs = {p["param_name"]: p["expect"] for p in self.params}
        expected = ExtractedMetadata(**kwargs)
        actual = setuppy.extract("setup.py", workdir=".")
        self.assertThat(str(actual), Equals(str(expected)))
        self.assertThat(actual, Equals(expected))


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
