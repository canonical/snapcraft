[Setup]
; NOTE: The value of AppId uniquely identifies this application. Do not use the same AppId value in installers for other applications.
; (To generate a new GUID, click Tools | Generate GUID inside the IDE.)
AppId={{05E40DED-CE0A-437E-B90C-25A32B47880F}
AppName=Snapcraft for Windows
AppVersion=3.7.2
;AppVerName=Snapcraft for Windows 3.7.2
AppPublisher=Canonical Ltd.
AppPublisherURL=https://snapcraft.io/
AppSupportURL=https://snapcraft.io/
AppUpdatesURL=https://snapcraft.io/
DefaultDirName={autopf}\Snapcraft for Windows
DisableProgramGroupPage=yes
LicenseFile=COPYING
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
SetupIconFile=misc/snapcraft.ico
Compression=lzma
SolidCompression=yes
WizardStyle=modern
OutputBaseFilename=snapcraft-installer
OutputDir=dist

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]

[Files]
Source: "dist\snapcraft.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "dist\multipass-0.8.0+win-win64.exe"; DestDir: "{tmp}";

[Icons]
Name: "{autoprograms}\Snapcraft for Windows"; Filename: "{app}\snapcraft.exe"

[Run]
Filename: "{tmp}\multipass-0.8.0+win-win64.exe"; Description: "Install Multipass"; Flags: postinstall runascurrentuser

