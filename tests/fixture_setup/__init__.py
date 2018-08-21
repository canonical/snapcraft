import socket

if hasattr(socket, "AF_UNIX"):
    from ._unix import FakeSnapd  # noqa: F401
from ._fixtures import (  # noqa: F401
    BzrRepo,
    CleanEnvironment,
    GitRepo,
    HgRepo,
    FakeAptBaseDependency,
    FakeAptCache,
    FakeAptCachePackage,
    FakeBaseEnvironment,
    FakeElf,
    FakeLXD,
    FakeMetadataExtractor,
    FakeProjectOptions,
    FakeParts,
    FakePartsServerRunning,
    FakePartsWiki,
    FakePartsWikiWithSlashes,
    FakePartsWikiWithSlashesRunning,
    FakePartsWikiOrigin,
    FakePartsWikiOriginRunning,
    FakePartsWikiRunning,
    FakePlugin,
    FakeServerRunning,
    FakeSnapcraftctl,
    FakeSnapcraftIsASnap,
    FakeSSOServerRunning,
    FakeStore,
    FakeStoreAPIServerRunning,
    FakeStoreSearchServerRunning,
    FakeStoreUploadServerRunning,
    FakeTerminal,
    SharedCache,
    SilentSnapProgress,
    SnapcraftYaml,
    StagingStore,
    SvnRepo,
    TempCWD,
    TempXDG,
    TestStore,
    WithoutSnapInstalled,
)
