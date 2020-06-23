# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

from snapcraft.internal.review_tools import errors


class TestSnapcraftException:

    scenarios = (
        (
            "ReviewError (linting error with link)",
            {
                "exception_class": errors.ReviewError,
                "kwargs": dict(
                    review_json={
                        "snap.v2_functional": {"error": {}, "warn": {}},
                        "snap.v2_lint": {
                            "error": {
                                "lint-snap-v2:lint_issue": {
                                    "link": "https://issue-link",
                                    "text": "(NEEDS REVIEW) linting message.",
                                }
                            },
                            "warn": {},
                        },
                        "snap.v2_security": {"error": {}, "warn": {}},
                    }
                ),
                "expected_details": dedent(
                    """\
                    Linting Issues:
                    - (NEEDS REVIEW) linting message. (Refer to https://issue-link)"""
                ),
            },
        ),
        (
            "ReviewError (linting warning with link)",
            {
                "exception_class": errors.ReviewError,
                "kwargs": dict(
                    review_json={
                        "snap.v2_functional": {"error": {}, "warn": {}},
                        "snap.v2_lint": {
                            "warn": {
                                "lint-snap-v2:lint_issue": {
                                    "link": "https://issue-link",
                                    "text": "(NEEDS REVIEW) linting message.",
                                }
                            },
                            "error": {},
                        },
                        "snap.v2_security": {"error": {}, "warn": {}},
                    }
                ),
                "expected_details": dedent(
                    """\
                    Linting Issues:
                    - (NEEDS REVIEW) linting message. (Refer to https://issue-link)"""
                ),
            },
        ),
        (
            "ReviewError (linting error without link)",
            {
                "exception_class": errors.ReviewError,
                "kwargs": dict(
                    review_json={
                        "snap.v2_functional": {"error": {}, "warn": {}},
                        "snap.v2_lint": {
                            "error": {
                                "lint-snap-v2:lint_issue": {
                                    "text": "(NEEDS REVIEW) linting message."
                                }
                            },
                            "warn": {},
                        },
                        "snap.v2_security": {"error": {}, "warn": {}},
                    }
                ),
                "expected_details": dedent(
                    """\
                    Linting Issues:
                    - (NEEDS REVIEW) linting message."""
                ),
            },
        ),
        (
            "ReviewError (linting warning without link)",
            {
                "exception_class": errors.ReviewError,
                "kwargs": dict(
                    review_json={
                        "snap.v2_functional": {"error": {}, "warn": {}},
                        "snap.v2_lint": {
                            "warn": {
                                "lint-snap-v2:lint_issue": {
                                    "text": "(NEEDS REVIEW) linting message."
                                }
                            },
                            "error": {},
                        },
                        "snap.v2_security": {"error": {}, "warn": {}},
                    }
                ),
                "expected_details": dedent(
                    """\
                    Linting Issues:
                    - (NEEDS REVIEW) linting message."""
                ),
            },
        ),
        (
            "ReviewError (security error with link)",
            {
                "exception_class": errors.ReviewError,
                "kwargs": dict(
                    review_json={
                        "snap.v2_functional": {"error": {}, "warn": {}},
                        "snap.v2_security": {
                            "error": {
                                "security-snap-v2:security_issue": {
                                    "link": "https://issue-link",
                                    "text": "(NEEDS REVIEW) security message.",
                                }
                            },
                            "warn": {},
                        },
                        "snap.v2_lint": {"error": {}, "warn": {}},
                    }
                ),
                "expected_details": dedent(
                    """\
                    Security Issues:
                    - (NEEDS REVIEW) security message. (Refer to https://issue-link)"""
                ),
            },
        ),
        (
            "ReviewError (security warning with link)",
            {
                "exception_class": errors.ReviewError,
                "kwargs": dict(
                    review_json={
                        "snap.v2_functional": {"error": {}, "warn": {}},
                        "snap.v2_security": {
                            "warn": {
                                "security-snap-v2:security_issue": {
                                    "link": "https://issue-link",
                                    "text": "(NEEDS REVIEW) security message.",
                                }
                            },
                            "error": {},
                        },
                        "snap.v2_lint": {"error": {}, "warn": {}},
                    }
                ),
                "expected_details": dedent(
                    """\
                    Security Issues:
                    - (NEEDS REVIEW) security message. (Refer to https://issue-link)"""
                ),
            },
        ),
        (
            "ReviewError (security error without link)",
            {
                "exception_class": errors.ReviewError,
                "kwargs": dict(
                    review_json={
                        "snap.v2_functional": {"error": {}, "warn": {}},
                        "snap.v2_security": {
                            "error": {
                                "security-snap-v2:security_issue": {
                                    "text": "(NEEDS REVIEW) security message."
                                }
                            },
                            "warn": {},
                        },
                        "snap.v2_lint": {"error": {}, "warn": {}},
                    }
                ),
                "expected_details": dedent(
                    """\
                    Security Issues:
                    - (NEEDS REVIEW) security message."""
                ),
            },
        ),
        (
            "ReviewError (security warning without link)",
            {
                "exception_class": errors.ReviewError,
                "kwargs": dict(
                    review_json={
                        "snap.v2_functional": {"error": {}, "warn": {}},
                        "snap.v2_security": {
                            "warn": {
                                "security-snap-v2:security_issue": {
                                    "text": "(NEEDS REVIEW) security message."
                                }
                            },
                            "error": {},
                        },
                        "snap.v2_lint": {"error": {}, "warn": {}},
                    }
                ),
                "expected_details": dedent(
                    """\
                    Security Issues:
                    - (NEEDS REVIEW) security message."""
                ),
            },
        ),
        (
            "ReviewError (functional error with link)",
            {
                "exception_class": errors.ReviewError,
                "kwargs": dict(
                    review_json={
                        "snap.v2_security": {"error": {}, "warn": {}},
                        "snap.v2_functional": {
                            "error": {
                                "functional-snap-v2:functional_issue": {
                                    "link": "https://issue-link",
                                    "text": "(NEEDS REVIEW) functional message.",
                                }
                            },
                            "warn": {},
                        },
                        "snap.v2_lint": {"error": {}, "warn": {}},
                    }
                ),
                "expected_details": dedent(
                    """\
                    Functional Issues:
                    - (NEEDS REVIEW) functional message. (Refer to https://issue-link)"""
                ),
            },
        ),
        (
            "ReviewError (functional warning with link)",
            {
                "exception_class": errors.ReviewError,
                "kwargs": dict(
                    review_json={
                        "snap.v2_security": {"error": {}, "warn": {}},
                        "snap.v2_functional": {
                            "warn": {
                                "functional-snap-v2:functional_issue": {
                                    "link": "https://issue-link",
                                    "text": "(NEEDS REVIEW) functional message.",
                                }
                            },
                            "error": {},
                        },
                        "snap.v2_lint": {"error": {}, "warn": {}},
                    }
                ),
                "expected_details": dedent(
                    """\
                    Functional Issues:
                    - (NEEDS REVIEW) functional message. (Refer to https://issue-link)"""
                ),
            },
        ),
        (
            "ReviewError (functional error without link)",
            {
                "exception_class": errors.ReviewError,
                "kwargs": dict(
                    review_json={
                        "snap.v2_security": {"error": {}, "warn": {}},
                        "snap.v2_functional": {
                            "error": {
                                "functional-snap-v2:functional_issue": {
                                    "text": "(NEEDS REVIEW) functional message."
                                }
                            },
                            "warn": {},
                        },
                        "snap.v2_lint": {"error": {}, "warn": {}},
                    }
                ),
                "expected_details": dedent(
                    """\
                    Functional Issues:
                    - (NEEDS REVIEW) functional message."""
                ),
            },
        ),
        (
            "ReviewError (functional warning without link)",
            {
                "exception_class": errors.ReviewError,
                "kwargs": dict(
                    review_json={
                        "snap.v2_security": {"error": {}, "warn": {}},
                        "snap.v2_functional": {
                            "warn": {
                                "functional-snap-v2:functional_issue": {
                                    "text": "(NEEDS REVIEW) functional message."
                                }
                            },
                            "error": {},
                        },
                        "snap.v2_lint": {"error": {}, "warn": {}},
                    }
                ),
                "expected_details": dedent(
                    """\
                    Functional Issues:
                    - (NEEDS REVIEW) functional message."""
                ),
            },
        ),
        (
            "ReviewError",
            {
                "exception_class": errors.ReviewError,
                "kwargs": dict(
                    review_json={
                        "snap.v2_security": {
                            "error": {},
                            "warn": {
                                "functional-snap-v2:security_issue": {
                                    "text": "(NEEDS REVIEW) security message."
                                }
                            },
                        },
                        "snap.v2_functional": {
                            "warn": {
                                "functional-snap-v2:functional_issue": {
                                    "text": "(NEEDS REVIEW) functional message."
                                }
                            },
                            "error": {},
                        },
                        "snap.v2_lint": {
                            "error": {},
                            "warn": {
                                "functional-snap-v2:lint_issue": {
                                    "text": "(NEEDS REVIEW) lint message."
                                }
                            },
                        },
                    }
                ),
                "expected_details": dedent(
                    """\
                    Functional Issues:
                    - (NEEDS REVIEW) functional message.

                    Linting Issues:
                    - (NEEDS REVIEW) lint message.

                    Security Issues:
                    - (NEEDS REVIEW) security message."""
                ),
            },
        ),
    )

    def test_snapcraft_exception_handling(
        self, exception_class, expected_details, kwargs
    ):
        exception = exception_class(**kwargs)

        assert exception.get_details() == expected_details
        assert exception.get_brief() == "Review Tools did not fully pass for this snap."
        assert (
            exception.get_resolution()
            == "Specific measures might need to be taken on the Snap Store before this snap can be fully accepted."
        )
        assert exception.get_docs_url() is None
        assert exception.get_reportable() is False
