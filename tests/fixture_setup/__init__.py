import os
import socket

if hasattr(socket, "AF_UNIX"):
    from ._unix import FakeSnapd  # noqa: F401
try:
    from ._unittests import (  # noqa: F401
        FakeElf,
        FakeExtension,
        FakeMetadataExtractor,
        FakePlugin,
        FakeProjectOptions,
        FakeSnapCommand,
        FakeSnapcraftctl,
        FakeMultipass,
        SilentSnapProgress,
    )
except ImportError as import_error:
    if os.path.exists(os.path.join(os.path.dirname(__file__), "..", "snapcraft")):
        raise import_error

from ._fixtures import (  # noqa: F401
    BzrRepo,
    CleanEnvironment,
    GitRepo,
    HgRepo,
    FakeBaseEnvironment,
    FakeParts,
    FakePartsServerRunning,
    FakePartsWiki,
    FakePartsWikiWithSlashes,
    FakePartsWikiWithSlashesRunning,
    FakePartsWikiOrigin,
    FakePartsWikiOriginRunning,
    FakePartsWikiRunning,
    FakeServerRunning,
    FakeSnapcraftIsASnap,
    FakeSSOServerRunning,
    FakeStore,
    FakeStoreAPIServerRunning,
    FakeStoreSearchServerRunning,
    FakeStoreUploadServerRunning,
    FakeTerminal,
    SharedCache,
    SnapcraftYaml,
    StagingStore,
    SvnRepo,
    TempCWD,
    TempXDG,
    TestStore,
    WithoutSnapInstalled,
)
