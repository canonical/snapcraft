[Setup]
AppId={{05E40DED-CE0A-437E-B90C-25A32B47880F}
AppName=Snapcraft (Preview) for Windows
AppVersion=VERSION
AppPublisher=Canonical Ltd.
AppPublisherURL=https://snapcraft.io/
AppSupportURL=https://snapcraft.io/
AppUpdatesURL=https://snapcraft.io/
DefaultDirName={autopf}\Snapcraft for Windows
DisableProgramGroupPage=yes
LicenseFile=..\COPYING
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
SetupIconFile=snapcraft.ico
Compression=lzma
SolidCompression=yes
WizardStyle=modern
OutputBaseFilename=snapcraft-installer
OutputDir=..\dist
ChangesEnvironment=yes

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: modifypath; Description: "Add snapcraft to the current user's PATH (Recommended)"

[Files]
Source: "..\dist\snapcraft.exe"; DestDir: "{app}"; Flags: ignoreversion

[Code]
const
  ModPathName = 'modifypath';
  ModPathType = 'user';

function ModPathDir(): TArrayOfString;
begin
  SetArrayLength(Result, 1);
  Result[0] := ExpandConstant('{app}');
end;
#include "modpath.iss"

procedure CurStepChanged(CurStep: TSetupStep);
var
  Success: Boolean;
begin
  Success := True;
  if CurStep = ssPostInstall then
  begin
    if WizardIsTaskSelected(ModPathName) then
      ModPath();
  end;
end;
