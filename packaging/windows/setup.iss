; RoboWorld Windows 安装器（Inno Setup 6）
;
; 编译：ISCC /DMyAppVersion=1.2.3 /DMyDistDir=<abs dist> packaging\windows\setup.iss
; 一般由 packaging\build_installer_win.ps1 代劳（它会从 support.__version__ 读出
; 版本号传进来）。
;
; 为什么需要安装器（而不只是发一个 exe）：exe 是 onefile 的绿色版，拷到哪都能跑，
; 但**没人替它告诉 Windows「.rwproj 归它打开」**。双击工程文件因此会弹
; 「你要如何打开这个文件」——用户眼里就是"装了软件却打不开工程"。
; 绿色版的自注册见 app/platform_win.py（写 HKCU）；本脚本面向安装场景。
;
; 三个值与代码同源，改代码必须改这里（由 tests/test_installer.py 静态守住）：
;   MyAppId        ← app.app_identity.APPLICATION_ID
;   ProjectProgId  ← app.app_identity.PROJECT_PROG_ID
;   ProjectType    ← app.app_identity.PROJECT_TYPE_NAME
;   扩展名列表     ← service.project.PROJECT_EXTENSIONS

#define MyAppId "com.openrobot.work.robotworld"
#define ProjectProgId "RoboWorld.Project.1"
#define ProjectType "RoboWorld Project"

#define MyAppName "RoboWorld"
#define MyAppExe "RoboWorld.exe"
#define MyAppPublisher "RoboWorld Contributors"
#define MyAppURL "https://github.com/cModules/RoboWorld"

; 版本号由构建脚本传入（唯一真源 support/__init__.py 的 __version__）。
; 直接双击编译时用它兜底——宁可显示 0.0.0，也不要在打包脚本里手写第四份版本号。
#ifndef MyAppVersion
  #define MyAppVersion "0.0.0"
#endif

; PyInstaller 的产出目录（相对本文件：packaging/windows → ../../dist）
#ifndef MyDistDir
  #define MyDistDir "..\..\dist"
#endif

; 安装包输出目录
#ifndef MyOutputDir
  #define MyOutputDir "..\..\dist"
#endif

[Setup]
; AppId 用 APPLICATION_ID 而不是另起一个 GUID：它是"这个应用是谁"的跨端答案，
; 与 AppUserModelID / bundle id 同口径。换过 AppId 的后果是旧版本卸载不掉、
; 新版本被当成另一个软件装两份。
AppId={#MyAppId}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppVerName={#MyAppName} {#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}/issues
AppUpdatesURL={#MyAppURL}/releases
VersionInfoVersion={#MyAppVersion}

; 默认装到当前用户（不需要管理员）。用户想装到 Program Files 时，
; PrivilegesRequiredOverridesAllowed 允许他在安装界面上提升到管理员。
DefaultDirName={autopf}\{#MyAppName}
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
DefaultGroupName={#MyAppName}
DisableDirPage=auto
DisableProgramGroupPage=auto
DisableWelcomePage=no

; 关键：**没有这一行，Windows 8 及以上不会采用新注册的文件关联**。
; 系统自 Vista 起记住"用户为某类型选过什么程序"，新装软件写进注册表也会被
; 那段记忆盖掉——表现为"装完双击还是用旧软件打开，重装也没用"。
; 设了它，安装器会把本次关联申报给系统的默认程序机制。
ChangesAssociations=yes

; 64 位系统用 Program Files；32 位上 {autopf} 自动落到 Program Files (x86)
ArchitecturesInstallIn64BitMode=x64compatible

OutputBaseFilename={#MyAppName}-Setup-{#MyAppVersion}
OutputDir={#MyOutputDir}
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
UninstallDisplayName={#MyAppName} {#MyAppVersion}
UninstallDisplayIcon={app}\{#MyAppExe}
; 没有 SetupIconFile：Inno 只认 .ico，而仓库里只有 logo.png / logo.icns。
; 放一个 png 会让编译直接失败，等补了 .ico 再启用这一行。
; SetupIconFile=..\logo.ico

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"
Name: "chinesesimplified"; MessagesFile: "compiler:Languages\ChineseSimplified.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "{#MyDistDir}\{#MyAppExe}"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\..\LICENSE"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\..\README.md"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExe}"
Name: "{group}\{cm:UninstallProgram,{#MyAppName}}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExe}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExe}"; Description: "{cm:LaunchProgram,{#MyAppName}}"; Flags: nowait postinstall skipifsilent

; --------------------------------------------------------------------------- #
; 文件关联
;
; HKA = 有管理员权限时 HKLM、否则 HKCU（与 PrivilegesRequired=lowest 配套）。
; 写死 HKCR/HKLM 的后果：非管理员安装会因权限不足静默失败，装完照样打不开。
;
; "%1" 必须带引号：路径含空格时不带引号会被截成两段，症状是"双击有的工程能开、
; 有的打不开"。
; --------------------------------------------------------------------------- #
[Registry]
; ProgID 本体：uninsdeletekey 会连同子键一起删（Inno 递归删）
Root: HKA; Subkey: "Software\Classes\{#ProjectProgId}"; ValueType: string; ValueName: ""; ValueData: "{#ProjectType}"; Flags: uninsdeletekey
Root: HKA; Subkey: "Software\Classes\{#ProjectProgId}\DefaultIcon"; ValueType: string; ValueName: ""; ValueData: "{app}\{#MyAppExe},0"
Root: HKA; Subkey: "Software\Classes\{#ProjectProgId}\shell\open\command"; ValueType: string; ValueName: ""; ValueData: """{app}\{#MyAppExe}"" ""%1"""

; 扩展名 → ProgID。三行必须覆盖 service.project.PROJECT_EXTENSIONS 的每一项
; （含历史别名 .core）——漏一个就是"这种文件双击没反应"。
; 卸载用 uninsdeletevalue：只删我们写的默认值，键本身留着。用 uninsdeletekey
; 会把用户后来改指给别的软件的关联一起删掉。
Root: HKA; Subkey: "Software\Classes\.rwproj"; ValueType: string; ValueName: ""; ValueData: "{#ProjectProgId}"; Flags: uninsdeletevalue
Root: HKA; Subkey: "Software\Classes\.robotproj"; ValueType: string; ValueName: ""; ValueData: "{#ProjectProgId}"; Flags: uninsdeletevalue
Root: HKA; Subkey: "Software\Classes\.core"; ValueType: string; ValueName: ""; ValueData: "{#ProjectProgId}"; Flags: uninsdeletevalue

; Applications 键：让本程序出现在"打开方式 → 选择其他应用"的列表里。
; 缺这一段时，若 .rwproj 已被别的程序占用，用户在打开方式里根本看不到
; RoboWorld——只能"查找其他应用"去 Program Files 里翻 exe。
Root: HKA; Subkey: "Software\Classes\Applications\{#MyAppExe}\shell\open\command"; ValueType: string; ValueName: ""; ValueData: """{app}\{#MyAppExe}"" ""%1"""; Flags: uninsdeletekey
Root: HKA; Subkey: "Software\Classes\Applications\{#MyAppExe}\DefaultIcon"; ValueType: string; ValueName: ""; ValueData: "{app}\{#MyAppExe},0"
Root: HKA; Subkey: "Software\Classes\Applications\{#MyAppExe}\SupportedTypes"; ValueType: string; ValueName: ".rwproj"; ValueData: ""; Flags: uninsdeletevalue
Root: HKA; Subkey: "Software\Classes\Applications\{#MyAppExe}\SupportedTypes"; ValueType: string; ValueName: ".robotproj"; ValueData: ""; Flags: uninsdeletevalue
Root: HKA; Subkey: "Software\Classes\Applications\{#MyAppExe}\SupportedTypes"; ValueType: string; ValueName: ".core"; ValueData: ""; Flags: uninsdeletevalue
