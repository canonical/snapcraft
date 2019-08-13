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
LicenseFile=..\COPYING
; TODO: multipass installer only supports admin installation
PrivilegesRequired=admin
;PrivilegesRequired=lowest
;PrivilegesRequiredOverridesAllowed=dialog
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
;Name: modifypathsystem; Description: "Add snapcraft to the system PATH for all users"; Flags: unchecked

[Files]
Source: "..\dist\snapcraft.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\dist\multipass-0.8.0+win-win64.exe"; DestDir: "{tmp}";

[Icons]
Name: "{autoprograms}\Snapcraft for Windows"; Filename: "{app}\snapcraft.exe"

[Run]
Filename: "{tmp}\multipass-0.8.0+win-win64.exe"; Description: "Install Multipass"; Flags: postinstall runascurrentuser

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