# -*- mode: python ; coding: utf-8 -*-

block_cipher = None


a = Analysis(
    ["snapcraft\\cli\\__main__.py"],
    pathex=[],
    binaries=[],
    datas=[
        ("extensions", os.path.join("share", "snapcraft", "extensions")),
        ("keyrings", os.path.join("share", "snapcraft", "keyrings")),
        ("schema", os.path.join("share", "snapcraft", "schema")),
    ],
    hiddenimports=[
        "cffi",
        "click",
        "configparser",
        "pkg_resources",
        "pymacaroons",
        "responses",
    ],
    hookspath=[],
    runtime_hooks=[os.path.join("windows", "hook-pbr.py")],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name="snapcraft",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    icon="windows/snapcraft.ico",
)
