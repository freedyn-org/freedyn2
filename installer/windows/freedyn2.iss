; FreeDyn 2 installer for Windows x64 (Inno Setup 6.3 or newer).
;
; Built by installer/windows/build.py from the Windows wheel, which first stages the files
; (StageDir: core, compiler, examples) and then runs
;   ISCC /DAppVersion=0.1.0 /DAppVersionNumeric=0.1.0.0 /DStageDir=... /DOutputDir=... /DRepoDir=... freedyn2.iss
;
; Per user by default ({localappdata}\Programs\FreeDyn2, no administrator rights), or for all users
; ({commonpf64}\FreeDyn2) when chosen in the dialog or with /ALLUSERS. A new version installs over
; the old one (same AppId). Independent of the legacy FreeDyn (other name, directory, file type ids).
;
; Silent:  FreeDyn2-<version>-setup.exe /VERYSILENT /SUPPRESSMSGBOXES /NORESTART /CURRENTUSER
;            [/DIR="..."] [/COMPONENTS="core,compiler,examples"] [/TASKS="path,assocfds,openyaml,claudedesktop,desktopicon"]
;          (/TASKS replaces the default selection, /MERGETASKS adds to it; prefix ! deselects)
;          unins000.exe /VERYSILENT /SUPPRESSMSGBOXES /NORESTART
; The uninstaller keeps the user data (%APPDATA%\freedyn2, %LOCALAPPDATA%\freedyn2).

#ifndef AppVersion
  #error Define AppVersion (ISCC /DAppVersion=0.1.0); installer/windows/build.py does
#endif
#ifndef AppVersionNumeric
  #define AppVersionNumeric AppVersion
#endif
#ifndef StageDir
  #define StageDir "..\..\build\installer\stage"
#endif
#ifndef OutputDir
  #define OutputDir "..\..\dist"
#endif
#ifndef RepoDir
  #define RepoDir "..\.."
#endif

#define AppName "freedyn2"  ; the brand (brand/README.md): always lower case, one word
#define AppURL "https://github.com/freedyn-org/freedyn2"
#define DocsURL "https://freedyn-org.github.io/freedyn2/"
#define GuiExe "{app}\bin\freedyn2-gui.exe"
#define CliExe "{app}\bin\freedyn2.exe"

[Setup]
; Fixed for all versions (upgrades, winget ProductCode "{D4AF9686-2D82-4C9F-9F17-CFE1861260DA}_is1"): never change.
AppId={{D4AF9686-2D82-4C9F-9F17-CFE1861260DA}
AppName={#AppName}
AppVersion={#AppVersion}
AppVerName={#AppName} (version {#AppVersion})
AppPublisher=The FreeDyn 2 contributors
AppPublisherURL={#AppURL}
AppSupportURL={#AppURL}/issues
AppUpdatesURL={#AppURL}/releases
AppCopyright=Copyright 2026 The FreeDyn 2 contributors (Apache-2.0)
VersionInfoVersion={#AppVersionNumeric}
VersionInfoProductName={#AppName}
VersionInfoDescription={#AppName} Setup
DefaultDirName={autopf}\FreeDyn2
DefaultGroupName={#AppName}
; the group of this version, not the one of an older installation ("FreeDyn 2", removed: InstallDelete)
UsePreviousGroup=no
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog commandline
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
; Windows 10 1809 or newer (WebView2 of the app window)
MinVersion=10.0.17763
UsePreviousAppDir=yes
UsePreviousTasks=yes
UsePreviousSetupType=yes
ChangesEnvironment=yes
ChangesAssociations=yes
CloseApplications=yes
CloseApplicationsFilter=*.exe,*.dll,*.pyd
RestartApplications=no
InfoBeforeFile=THIRD-PARTY-NOTICES.txt
OutputDir={#OutputDir}
OutputBaseFilename=FreeDyn2-{#AppVersion}-setup
Compression=lzma2/max
SolidCompression=yes
LZMAUseSeparateProcess=yes
WizardStyle=modern
UninstallDisplayName={#AppName}
#ifdef HaveIcon
SetupIconFile={#StageDir}\core\freedyn2.ico
UninstallDisplayIcon={app}\freedyn2.ico
#else
UninstallDisplayIcon={#GuiExe}
#endif

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Types]
Name: "full"; Description: "Full installation"
Name: "compact"; Description: "Compact installation (no C compiler, no examples)"
Name: "custom"; Description: "Custom installation"; Flags: iscustom

[Components]
Name: "core"; Description: "freedyn2 (solver, app, viewer, MCP server, Python runtime)"; Types: full compact custom; Flags: fixed
Name: "compiler"; Description: "C compiler for model plugins (Zig): models with expression forces or drivers"; Types: full
Name: "examples"; Description: "Example models"; Types: full

[Tasks]
Name: "path"; Description: "Add the freedyn2 command to the PATH"; GroupDescription: "Command line:"
Name: "assocfds"; Description: "Open FreeDyn model files (.fds) with freedyn2"; GroupDescription: "File types:"; Flags: unchecked
Name: "openyaml"; Description: "Add ""Open with freedyn2"" to .yaml files"; GroupDescription: "File types:"; Flags: unchecked
Name: "claudedesktop"; Description: "Register the freedyn2 MCP server with Claude Desktop"; GroupDescription: "Assistants:"; Flags: unchecked
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[InstallDelete]
; An upgrade replaces the program completely (no packages of the old version left, the compiler
; gone when deselected). Only directories of the program: the user data is elsewhere.
Type: filesandordirs; Name: "{app}\python"
Type: filesandordirs; Name: "{app}\bin"
Type: filesandordirs; Name: "{app}\licenses"
Type: filesandordirs; Name: "{app}\examples"
; the shortcuts of versions before the rename to freedyn2 (start menu folder and desktop "FreeDyn 2"):
; only these files, then the folder if it is empty
Type: files; Name: "{autoprograms}\FreeDyn 2\FreeDyn 2.lnk"
Type: files; Name: "{autoprograms}\FreeDyn 2\FreeDyn 2 Command Prompt.lnk"
Type: files; Name: "{autoprograms}\FreeDyn 2\FreeDyn 2 Documentation.url"
Type: files; Name: "{autoprograms}\FreeDyn 2\FreeDyn 2 Examples.lnk"
Type: dirifempty; Name: "{autoprograms}\FreeDyn 2"
Type: files; Name: "{autodesktop}\FreeDyn 2.lnk"

[Files]
Source: "{#StageDir}\core\*"; DestDir: "{app}"; Components: core; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "{#StageDir}\compiler\*"; DestDir: "{app}"; Components: compiler; Flags: ignoreversion recursesubdirs createallsubdirs skipifsourcedoesntexist
Source: "{#StageDir}\examples\*"; DestDir: "{app}"; Components: examples; Flags: ignoreversion recursesubdirs createallsubdirs skipifsourcedoesntexist
Source: "freedyn2-prompt.cmd"; DestDir: "{app}\bin"; Components: core; Flags: ignoreversion
Source: "THIRD-PARTY-NOTICES.txt"; DestDir: "{app}"; Components: core; Flags: ignoreversion
Source: "{#RepoDir}\LICENSE"; DestDir: "{app}"; DestName: "LICENSE.txt"; Components: core; Flags: ignoreversion
Source: "{#RepoDir}\NOTICE"; DestDir: "{app}"; DestName: "NOTICE.txt"; Components: core; Flags: ignoreversion

[Icons]
; Start menu folder "freedyn2" (no uninstall entry: Settings > Apps uninstalls)
#ifdef HaveIcon
Name: "{group}\freedyn2"; Filename: "{#GuiExe}"; IconFilename: "{app}\freedyn2.ico"; Comment: "Multibody dynamics: open, run and view models"
Name: "{group}\freedyn2 command prompt"; Filename: "{cmd}"; Parameters: "/k """"{app}\bin\freedyn2-prompt.cmd"" --home"""; IconFilename: "{app}\freedyn2.ico"; Comment: "A command prompt with the freedyn2 command"
Name: "{autodesktop}\freedyn2"; Filename: "{#GuiExe}"; IconFilename: "{app}\freedyn2.ico"; Tasks: desktopicon
#else
Name: "{group}\freedyn2"; Filename: "{#GuiExe}"; Comment: "Multibody dynamics: open, run and view models"
Name: "{group}\freedyn2 command prompt"; Filename: "{cmd}"; Parameters: "/k """"{app}\bin\freedyn2-prompt.cmd"" --home"""; Comment: "A command prompt with the freedyn2 command"
Name: "{autodesktop}\freedyn2"; Filename: "{#GuiExe}"; Tasks: desktopicon
#endif
Name: "{group}\freedyn2 documentation"; Filename: "{#DocsURL}"
Name: "{group}\freedyn2 examples"; Filename: "{app}\examples"; Components: examples

[Registry]
; HKA: HKCU\Software\Classes for a per-user installation, HKLM for all users.
; .fds (task assocfds): freedyn2 as the default program and in "Open with"
Root: HKA; Subkey: "Software\Classes\FreeDyn2.fds"; ValueType: string; ValueName: ""; ValueData: "FreeDyn model"; Flags: uninsdeletekey; Tasks: assocfds
#ifdef HaveIcon
Root: HKA; Subkey: "Software\Classes\FreeDyn2.fds\DefaultIcon"; ValueType: string; ValueName: ""; ValueData: "{app}\freedyn2.ico"; Tasks: assocfds
#endif
Root: HKA; Subkey: "Software\Classes\FreeDyn2.fds\shell\open\command"; ValueType: string; ValueName: ""; ValueData: """{#GuiExe}"" ""%1"""; Tasks: assocfds
Root: HKA; Subkey: "Software\Classes\.fds\OpenWithProgids"; ValueType: string; ValueName: "FreeDyn2.fds"; ValueData: ""; Flags: uninsdeletevalue; Tasks: assocfds
Root: HKA; Subkey: "Software\Classes\.fds"; ValueType: string; ValueName: ""; ValueData: "FreeDyn2.fds"; Flags: uninsdeletevalue; Tasks: assocfds
; .yaml, .yml (task openyaml): a context menu entry "Open with freedyn2", the default program unchanged
Root: HKA; Subkey: "Software\Classes\SystemFileAssociations\.yaml\shell\FreeDyn2"; ValueType: string; ValueName: ""; ValueData: "Open with freedyn2"; Flags: uninsdeletekey; Tasks: openyaml
Root: HKA; Subkey: "Software\Classes\SystemFileAssociations\.yaml\shell\FreeDyn2\command"; ValueType: string; ValueName: ""; ValueData: """{#GuiExe}"" ""%1"""; Tasks: openyaml
Root: HKA; Subkey: "Software\Classes\SystemFileAssociations\.yml\shell\FreeDyn2"; ValueType: string; ValueName: ""; ValueData: "Open with freedyn2"; Flags: uninsdeletekey; Tasks: openyaml
Root: HKA; Subkey: "Software\Classes\SystemFileAssociations\.yml\shell\FreeDyn2\command"; ValueType: string; ValueName: ""; ValueData: """{#GuiExe}"" ""%1"""; Tasks: openyaml
#ifdef HaveIcon
Root: HKA; Subkey: "Software\Classes\SystemFileAssociations\.yaml\shell\FreeDyn2"; ValueType: string; ValueName: "Icon"; ValueData: "{app}\freedyn2.ico"; Tasks: openyaml
Root: HKA; Subkey: "Software\Classes\SystemFileAssociations\.yml\shell\FreeDyn2"; ValueType: string; ValueName: "Icon"; ValueData: "{app}\freedyn2.ico"; Tasks: openyaml
#endif

[Run]
; Claude Desktop keeps its configuration per user: run as the user who started Setup
Filename: "{#CliExe}"; Parameters: "setup-mcp --client claude-desktop"; StatusMsg: "Registering freedyn2 with Claude Desktop..."; Flags: runhidden waituntilterminated runasoriginaluser; Tasks: claudedesktop
Filename: "{#GuiExe}"; Description: "Start freedyn2"; Flags: nowait postinstall skipifsilent runasoriginaluser

[UninstallRun]
Filename: "{#CliExe}"; Parameters: "setup-mcp --client claude-desktop --remove"; Flags: runhidden waituntilterminated; Tasks: claudedesktop; RunOnceId: "RemoveClaudeDesktop"

[UninstallDelete]
; files the program wrote into its own directory (bytecode of a per-user installation)
Type: filesandordirs; Name: "{app}\python"
Type: filesandordirs; Name: "{app}\bin"
Type: dirifempty; Name: "{app}"

[Code]
{ The PATH: the user's (HKCU\Environment) for a per-user installation, the system's for all users.
  Entries compare case-insensitively, without quotes and trailing backslashes; empty entries are
  dropped, the others kept in their order. ChangesEnvironment=yes broadcasts WM_SETTINGCHANGE. }

const
  SystemEnvKey = 'SYSTEM\CurrentControlSet\Control\Session Manager\Environment';
  UserEnvKey = 'Environment';

function EnvRootKey: Integer;
begin
  if IsAdminInstallMode then
    Result := HKEY_LOCAL_MACHINE
  else
    Result := HKEY_CURRENT_USER;
end;

function EnvSubKey: String;
begin
  if IsAdminInstallMode then
    Result := SystemEnvKey
  else
    Result := UserEnvKey;
end;

function NormalizePathEntry(S: String): String;
begin
  Result := Trim(S);
  if (Length(Result) >= 2) and (Result[1] = '"') and (Result[Length(Result)] = '"') then
    Result := Copy(Result, 2, Length(Result) - 2);
  while (Length(Result) > 3) and (Result[Length(Result)] = '\') do
    Result := Copy(Result, 1, Length(Result) - 1);
  Result := Lowercase(Result);
end;

{ Add (Add = True) or remove Dir from the PATH; writes the registry only if something changes. }
procedure UpdatePath(Dir: String; Add: Boolean);
var
  OldPath, NewPath, Entry, Rest: String;
  P: Integer;
  Found: Boolean;
begin
  if not RegQueryStringValue(EnvRootKey, EnvSubKey, 'Path', OldPath) then
    OldPath := '';
  NewPath := '';
  Found := False;
  Rest := OldPath;
  while Rest <> '' do
  begin
    P := Pos(';', Rest);
    if P > 0 then
    begin
      Entry := Copy(Rest, 1, P - 1);
      Delete(Rest, 1, P);
    end
    else
    begin
      Entry := Rest;
      Rest := '';
    end;
    if Trim(Entry) = '' then
      Continue;
    if NormalizePathEntry(Entry) = NormalizePathEntry(Dir) then
    begin
      Found := True;
      if not Add then
        Continue;
    end;
    if NewPath <> '' then
      NewPath := NewPath + ';';
    NewPath := NewPath + Entry;
  end;
  if Add = Found then
    Exit;  { already there, or nothing to remove }
  if Add then
  begin
    if NewPath <> '' then
      NewPath := NewPath + ';';
    NewPath := NewPath + Dir;
  end;
  if NewPath = '' then
  begin
    if not RegDeleteValue(EnvRootKey, EnvSubKey, 'Path') then
      Log('PATH: could not delete the empty value');
  end
  else if not RegWriteExpandStringValue(EnvRootKey, EnvSubKey, 'Path', NewPath) then
    Log('PATH: could not write ' + NewPath)
  else if Add then
    Log('PATH: added ' + Dir)
  else
    Log('PATH: removed ' + Dir);
end;

procedure CurStepChanged(CurStep: TSetupStep);
begin
  if CurStep = ssPostInstall then
    { also on an upgrade with the task now deselected: then the entry goes }
    UpdatePath(ExpandConstant('{app}\bin'), WizardIsTaskSelected('path'));
end;

procedure CurUninstallStepChanged(CurUninstallStep: TUninstallStep);
begin
  if CurUninstallStep = usUninstall then
    UpdatePath(ExpandConstant('{app}\bin'), False);
end;
