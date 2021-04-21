# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2020 Canonical Ltd
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

# Attributes that the jsonschema from the Snap Store originally require but
# Snapcraft does not have been commented out from this schema originally
# imported from
# https://dashboard.snapcraft.io/docs/v2/en/snaps.html#snap-channel-map

from typing import Any, Dict

# Version 14, found at: https://dashboard.snapcraft.io/docs/v2/en/snaps.html#snap-releases
RELEASES_JSONSCHEMA: Dict[str, Any] = {
    "properties": {
        "releases": {
            "items": {
                "properties": {
                    "architecture": {"introduced_at": 1, "type": "string"},
                    "branch": {"introduced_at": 1, "type": ["string", "null"]},
                    "channel": {
                        "description": "The channel name for this release.",
                        "introduced_at": 8,
                        "type": "string",
                    },
                    "expiration-date": {
                        "description": "The date when this release expires, in ISO 8601 format. If null, the release does not expire.",
                        "format": "date-time",
                        "introduced_at": 8,
                        "type": ["string", "null"],
                    },
                    "progressive": {
                        "introduced_at": 4,
                        "properties": {
                            "current-percentage": {
                                "introduced_at": 14,
                                "type": ["number", "null"],
                            },
                            "paused": {"type": ["boolean", "null"]},
                            "percentage": {"type": ["number", "null"]},
                        },
                        "required": ["paused", "percentage", "current-percentage"],
                        "type": "object",
                    },
                    "revision": {"introduced_at": 1, "type": ["integer", "null"]},
                    "risk": {"introduced_at": 1, "type": "string"},
                    "track": {"introduced_at": 1, "type": "string"},
                    "when": {
                        "format": "date-time",
                        "introduced_at": 1,
                        "type": "string",
                    },
                },
                "required": [
                    "architecture",
                    "branch",
                    "revision",
                    "risk",
                    "track",
                    "when",
                ],
                "type": "object",
            },
            "minItems": 0,
            "type": "array",
        },
        "revisions": {
            "items": {
                "properties": {
                    "architectures": {
                        "introduced_at": 1,
                        "items": {"type": "string"},
                        "minItems": 1,
                        "type": "array",
                    },
                    "attributes": {"introduced_at": 2, "type": "object"},
                    "base": {"introduced_at": 1, "type": ["string", "null"]},
                    "build_url": {"introduced_at": 1, "type": ["string", "null"]},
                    "confinement": {
                        "enum": ["strict", "classic", "devmode"],
                        "introduced_at": 1,
                        "type": "string",
                    },
                    "created_at": {
                        "format": "date-time",
                        "introduced_at": 1,
                        "type": "string",
                    },
                    "epoch": {"introduced_at": 1, "type": "object"},
                    "grade": {
                        "enum": ["stable", "devel"],
                        "introduced_at": 1,
                        "type": "string",
                    },
                    "revision": {"introduced_at": 1, "type": "integer"},
                    "sha3-384": {"introduced_at": 1, "type": "string"},
                    "size": {"introduced_at": 1, "type": "integer"},
                    "status": {
                        "enum": [
                            "Published",
                            "Unpublished",
                            "ManualReviewPending",
                            "NeedsInformation",
                            "AutomaticallyRejected",
                            "Rejected",
                        ],
                        "introduced_at": 1,
                        "type": "string",
                    },
                    "version": {"introduced_at": 1, "type": "string"},
                },
                "required": [
                    "architectures",
                    "build_url",
                    "confinement",
                    "created_at",
                    "grade",
                    "revision",
                    "sha3-384",
                    "size",
                    "status",
                    "version",
                ],
                "type": "object",
            },
            "minItems": 0,
            "type": "array",
        },
    },
    "required": ["releases", "revisions"],
    "type": "object",
}


CHANNEL_MAP_JSONSCHEMA: Dict[str, Any] = {
    "properties": {
        "channel-map": {
            "items": {
                "properties": {
                    "architecture": {"type": "string"},
                    "channel": {
                        "description": 'The channel name, including "latest/" for the latest track.',
                        "type": "string",
                    },
                    "expiration-date": {
                        "description": "The date when this release expires, in RFC 3339 format. If null, the release does not expire.",
                        "format": "date-time",
                        "type": ["string", "null"],
                    },
                    "progressive": {
                        "properties": {
                            "paused": {"type": ["boolean", "null"]},
                            "percentage": {"type": ["number", "null"]},
                            "current-percentage": {"type": ["number", "null"]},
                        },
                        "required": ["paused", "percentage", "current-percentage"],
                        "type": "object",
                    },
                    "revision": {"type": "integer"},
                    "when": {
                        "description": "The date when this release was made, in RFC 3339 format.",
                        "format": "date-time",
                        "type": "string",
                    },
                },
                "required": [
                    "architecture",
                    "channel",
                    "expiration-date",
                    "progressive",
                    "revision",
                    # "when"
                ],
                "type": "object",
            },
            "minItems": 0,
            "type": "array",
        },
        "revisions": {
            "items": {
                "properties": {
                    "architectures": {
                        "items": {"type": "string"},
                        "minItems": 1,
                        "type": "array",
                    },
                    "attributes": {"type": "object"},
                    "base": {"type": ["string", "null"]},
                    "build-url": {"type": ["string", "null"]},
                    "confinement": {
                        "enum": ["strict", "classic", "devmode"],
                        "type": "string",
                    },
                    "created-at": {"format": "date-time", "type": "string"},
                    "epoch": {
                        "properties": {
                            "read": {
                                "items": {"type": "integer"},
                                "minItems": 1,
                                "type": ["array", "null"],
                            },
                            "write": {
                                "items": {"type": "integer"},
                                "minItems": 1,
                                "type": ["array", "null"],
                            },
                        },
                        "required": ["read", "write"],
                        "type": "object",
                    },
                    "grade": {"enum": ["stable", "devel"], "type": "string"},
                    "revision": {"type": "integer"},
                    "sha3-384": {"type": "string"},
                    "size": {"type": "integer"},
                    "version": {"type": "string"},
                },
                "required": [
                    "architectures",
                    # "attributes",
                    # "base",
                    # "build-url",
                    # "confinement",
                    # "created-at",
                    # "epoch",
                    # "grade",
                    "revision",
                    # "sha3-384",
                    # "size",
                    # "status",
                    "version",
                ],
                "type": "object",
            },
            "minItems": 0,
            "type": "array",
        },
        "snap": {
            "description": "Metadata about the requested snap.",
            "introduced_at": 6,
            "properties": {
                "channels": {
                    "description": "The list of most relevant channels for this snap. Branches are only included if there is a release for it.",
                    "introduced_at": 9,
                    "items": {
                        "description": "A list of channels and their metadata for the requested snap.",
                        "properties": {
                            "branch": {
                                "description": "The branch name for this channel, can be null.",
                                "type": ["string", "null"],
                            },
                            "fallback": {
                                "description": "The name of the channel that this channel would fall back to if there were no releases in it. If null, this channel has no fallback channel.",
                                "type": ["string", "null"],
                            },
                            "name": {
                                "description": 'The channel name, including "latest/" for the latest track.',
                                "type": "string",
                            },
                            "risk": {
                                "description": "The risk name for this channel.",
                                "type": "string",
                            },
                            "track": {
                                "description": "The track name for this channel.",
                                "type": "string",
                            },
                        },
                        "required": ["name", "track", "risk", "branch", "fallback"],
                        "type": "object",
                    },
                    "minItems": 1,
                    "type": "array",
                },
                "default-track": {
                    "description": "The default track name for this snap. If no default track is set, this value is null.",
                    "type": ["string", "null"],
                },
                "id": {
                    "description": "The snap ID for this snap package.",
                    "type": "string",
                },
                "name": {"description": "The snap package name.", "type": "string"},
                "private": {
                    "description": "Whether this snap is private or not.",
                    "type": "boolean",
                },
                "tracks": {
                    "description": "An ordered list of most relevant tracks for this snap.",
                    "introduced_at": 9,
                    "items": {
                        "description": "An ordered list of tracks and their metadata for this snap.",
                        "properties": {
                            "creation-date": {
                                "description": "The track creation date, in ISO 8601 format.",
                                "format": "date-time",
                                "type": ["string", "null"],
                            },
                            "name": {
                                "description": "The track name.",
                                "type": "string",
                            },
                            "version-pattern": {
                                "description": "A Python regex to validate the versions being released to this track. If null, no validation is enforced.",
                                "type": ["string", "null"],
                            },
                        },
                        # pattern is documented as required but is not returned,
                        # version-pattern is returned instead.
                        "required": ["name", "creation-date", "version-pattern"],
                        "type": "object",
                    },
                    "minItems": 1,
                    "type": "array",
                },
            },
            "required": [
                # "id",
                "channels",
                # "default-track",
                "name",
                # "private",
                # "tracks"
            ],
            "type": "object",
        },
    },
    "required": ["channel-map", "revisions", "snap"],
    "type": "object",
}

# Version 27, found at https://dashboard.snapcraft.io/docs/v2/en/tokens.html#api-tokens-whoami
WHOAMI_JSONSCHEMA: Dict[str, Any] = {
    "properties": {
        "account": {
            "properties": {
                "email": {
                    "description": "The primary email for the user",
                    "type": "string",
                },
                "id": {
                    "description": "The store account ID for the user",
                    "type": "string",
                },
                "name": {
                    "description": "The display name for the user",
                    "type": "string",
                },
                "username": {
                    "description": "The store username for the user",
                    "type": "string",
                },
            },
            "required": ["email", "id", "name", "username"],
            "type": "object",
        },
        "channels": {
            "oneOf": [
                {
                    "description": "A list of channels to restrict the macaroon to, allows wildcards in the fnmatch format (https://docs.python.org/3/library/fnmatch.html)",
                    "items": {"description": "Valid channel name.", "type": "string"},
                    "minItems": 1,
                    "type": "array",
                    "uniqueItems": True,
                },
                {"type": "null"},
            ]
        },
        "errors": {
            "items": {
                "properties": {
                    "code": {"type": "string"},
                    "extra": {"additionalProperties": True, "type": "object"},
                    "message": {"type": "string"},
                },
                "required": ["code", "message", "extra"],
                "type": "object",
            },
            "type": "array",
        },
        "expires": {
            "oneOf": [
                {
                    "description": "A timestamp (string formatted per ISO8601) after which the macaroon should not be valid",
                    "format": "date-time",
                    "type": "string",
                },
                {"type": "null"},
            ]
        },
        "packages": {
            "oneOf": [
                {
                    "description": 'A list of packages to restrict the macaroon to. Those can be defined in two ways: 1- by name: each item in the list should be a json dict like `{"name": "the-name"}`, or 2- by snap_id: each item in the list should be a json dict like `{"snap_id": "some-snap-id-1234"}`.',
                    "items": {
                        "anyOf": [{"required": ["snap_id"]}, {"required": ["name"]}],
                        "description": "Package identifier: a dict with at least name or snap_id keys.",
                        "properties": {
                            "name": {"type": "string"},
                            "snap_id": {"type": "string"},
                        },
                        "type": "object",
                    },
                    "minItems": 1,
                    "type": "array",
                    "uniqueItems": True,
                },
                {"type": "null"},
            ]
        },
        "permissions": {
            "oneOf": [
                {
                    "description": "A list of permissions to restrict the macaroon to",
                    "items": {
                        "description": "A valid token permission.",
                        "enum": [
                            "edit_account",
                            "modify_account_key",
                            "package_access",
                            "package_manage",
                            "package_metrics",
                            "package_purchase",
                            "package_push",
                            "package_register",
                            "package_release",
                            "package_update",
                            "package_upload",
                            "package_upload_request",
                            "store_admin",
                            "store_review",
                        ],
                        "type": "string",
                    },
                    "minItems": 1,
                    "type": "array",
                    "uniqueItems": True,
                },
                {"type": "null"},
            ]
        },
        "store_ids": {
            "oneOf": [
                {
                    "description": "A list of store IDs to restrict the macaroon to",
                    "items": {"description": "A store ID.", "type": "string"},
                    "minItems": 1,
                    "type": "array",
                    "uniqueItems": True,
                },
                {"type": "null"},
            ]
        },
    },
    "required": ["account", "channels", "packages", "permissions"],
    "type": "object",
}

# https://dashboard.snapcraft.io/docs/v2/en/validation-sets.html
BUILD_ASSERTION_JSONSCHEMA: Dict[str, Any] = {
    "properties": {
        "account-id": {
            "description": 'The "account-id" assertion header',
            "type": "string",
        },
        "authority-id": {
            "description": 'The "authority-id" assertion header',
            "type": "string",
        },
        "name": {"description": 'The "name" assertion header', "type": "string"},
        "revision": {
            "description": 'The "revision" assertion header',
            "type": "string",
        },
        "sequence": {
            "description": 'The "sequence" assertion header',
            "type": "string",
        },
        "series": {"description": 'The "series" assertion header', "type": "string"},
        "snaps": {
            "items": {
                "description": "List of snaps in a Validation Set assertion",
                "properties": {
                    "id": {
                        "description": "Snap ID",
                        "maxLength": 100,
                        "type": "string",
                    },
                    "name": {
                        "description": "Snap name",
                        "maxLength": 100,
                        "type": "string",
                    },
                    "presence": {
                        "description": "Snap presence",
                        "enum": ["required", "optional", "invalid"],
                        "type": "string",
                    },
                    "revision": {"description": "Snap revision", "type": "string"},
                },
                "required": ["name"],
                "type": "object",
            },
            "minItems": 1,
            "type": "array",
        },
        "timestamp": {
            "description": 'The "timestamp" assertion header',
            "type": "string",
        },
        "type": {
            "const": "validation-set",
            "description": 'The "type" assertion header',
            "type": "string",
        },
    },
    "required": [
        "type",
        "authority-id",
        "series",
        "account-id",
        "name",
        "sequence",
        # "revision",
        "timestamp",
        "snaps",
    ],
    "type": "object",
}
