
# All bases recognized by snapcraft
BASES = {"core", "core18", "core20", "core22", "devel"}
# Bases no longer supported by the current version of snapcraft
ESM_BASES = {"core", "core18"}
# Bases handled by the legacy snapcraft codebase
LEGACY_BASES = {"core20"}
# Bases handled by the current snapcraft codebase
CURRENT_BASES = BASES - ESM_BASES - LEGACY_BASES
