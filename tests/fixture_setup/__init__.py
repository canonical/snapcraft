import os
import socket

if hasattr(socket, "AF_UNIX"):
    from ._unix import FakeSnapd  # noqa: F401
try:
    from ._unittests import (  # noqa: F401
        FakeElf,
        FakeExtension,
        FakeMetadataExtractor,
        FakeMultipass,
        FakePlugin,
        FakeProjectOptions,
        FakeSnapCommand,
        FakeSnapcraftctl,
    )
except ImportError as import_error:
    if os.path.exists(os.path.join(os.path.dirname(__file__), "..", "snapcraft")):
        raise import_error

from ._fixtures import (  # noqa: F401
    BzrRepo,
    CleanEnvironment,
    FakeBaseEnvironment,
    FakeParts,
    FakePartsServerRunning,
    FakePartsWiki,
    FakePartsWikiOrigin,
    FakePartsWikiOriginRunning,
    FakePartsWikiRunning,
    FakePartsWikiWithSlashes,
    FakePartsWikiWithSlashesRunning,
    FakeServerRunning,
    FakeSnapcraftIsASnap,
    FakeSSOServerRunning,
    FakeStore,
    FakeStoreAPIServerRunning,
    FakeStoreSearchServerRunning,
    FakeStoreUploadServerRunning,
    FakeTerminal,
    GitRepo,
    HgRepo,
    SharedCache,
    SnapcraftYaml,
    StagingStore,
    SvnRepo,
    TempCWD,
    TempXDG,
    TestStore,
    WithoutSnapInstalled,
)
