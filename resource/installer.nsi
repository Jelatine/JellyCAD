Unicode true

!define PRODUCT_NAME "JellyCAD"
!define PRODUCT_SHORTCUT_NAME "JellyCAD_${PRODUCT_VERSION}"
!define INSTALL_ICON "..\resource\favicon.ico"
!define UNINSTALL_ICON "uninstall_logo.ico"
!define SOURCE_DIR "bin"
!define PRODUCT_PUBLISHER "lijianbin"
!define PRODUCT_WEB_SITE "https://github.com/Jelatine/JellyCAD"
!define BRANDING_TEXT "${PRODUCT_WEB_SITE}"
!define OUT_FILE "${PRODUCT_NAME}_Setup_${PRODUCT_VERSION}.exe"
!define PRODUCT_DIR_REGKEY "Software\Microsoft\Windows\CurrentVersion\AppPaths\${PRODUCT_NAME}.exe"
!define PRODUCT_UNINST_KEY "Software\Microsoft\Windows\CurrentVersion\Uninstall\${PRODUCT_NAME}"
!define PRODUCT_UNINST_ROOT_KEY "HKLM"
!define PRODUCT_STARTMENU_REGVAL "NSIS:StartMenuDir"

; Compressor
SetCompressor /SOLID lzma
SetCompressorDictSize 32

; ------ MUI Begin ------
!include "MUI.nsh"

; MUI Settings
!define MUI_ICON "${INSTALL_ICON}"

; Welcome Page
!insertmacro MUI_PAGE_WELCOME

; LICENSE Page
!insertmacro MUI_PAGE_LICENSE "..\LICENSE"

; Install Selection Page
!insertmacro MUI_PAGE_DIRECTORY
var ICONS_GROUP
!define MUI_STARTMENUPAGE_NODISABLE
!define MUI_STARTMENUPAGE_DEFAULTFOLDER "${PRODUCT_NAME}"
!define MUI_STARTMENUPAGE_REGISTRY_ROOT "${PRODUCT_UNINST_ROOT_KEY}"
!define MUI_STARTMENUPAGE_REGISTRY_KEY "${PRODUCT_UNINST_KEY}"
!define MUI_STARTMENUPAGE_REGISTRY_VALUENAME "${PRODUCT_STARTMENU_REGVAL}"
!insertmacro MUI_PAGE_STARTMENU Application $ICONS_GROUP
!insertmacro MUI_PAGE_INSTFILES
!define MUI_FINISHPAGE_RUN "$INSTDIR\${PRODUCT_NAME}.exe"
!insertmacro MUI_PAGE_FINISH

!insertmacro MUI_UNPAGE_INSTFILES

!insertmacro MUI_LANGUAGE "English"
!insertmacro MUI_LANGUAGE "SimpChinese"
  
; ------ MUI End ------



Name "${PRODUCT_NAME} ${PRODUCT_VERSION}"
OutFile "${OUT_FILE}"
InstallDir "$APPDATA\${PRODUCT_NAME}"
InstallDirRegKey HKLM "${PRODUCT_UNINST_KEY}""UninstallString"
ShowInstDetails show
ShowUnInstDetails show
BrandingText "${BRANDING_TEXT}"

Var UNINSTALL_PROG
Var OLD_VER
Var OLD_PATH
; Old Version
Function .onInit
  !insertmacro MUI_LANGDLL_DISPLAY

  ClearErrors
  ReadRegStr $UNINSTALL_PROG ${PRODUCT_UNINST_ROOT_KEY} ${PRODUCT_UNINST_KEY} "UninstallString"
  IfErrors  done

  ReadRegStr $OLD_VER ${PRODUCT_UNINST_ROOT_KEY} ${PRODUCT_UNINST_KEY} "DisplayVersion"
  MessageBox MB_YESNOCANCEL|MB_ICONQUESTION \
    "It is detected that ${PRODUCT_NAME} $OLD_VER has been installed on this machine.\
    $\n$\nUninstall the installed version first?" \
      /SD IDYES \
      IDYES uninstall \
      IDNO done
  Abort

uninstall:
  StrCpy $OLD_PATH $UNINSTALL_PROG -10


  ExecWait '"$UNINSTALL_PROG" /S _?=$OLD_PATH' $0
  DetailPrint "uninst.exe returned $0"
  Delete "$UNINSTALL_PROG"
  RMDir $OLD_PATH


done:
FunctionEnd

Section "MainSection" SEC01  
  SetOutPath "$INSTDIR"
  SetOverwrite try
  File /r "${SOURCE_DIR}\*"  
SectionEnd

Section -AdditionalIcons
  !insertmacro MUI_STARTMENU_WRITE_BEGIN Application
  CreateDirectory "$SMPROGRAMS\$ICONS_GROUP"
  CreateShortCut "$SMPROGRAMS\$ICONS_GROUP\${PRODUCT_SHORTCUT_NAME}.lnk" "$INSTDIR\${PRODUCT_NAME}.exe"
  CreateShortCut "$SMPROGRAMS\$ICONS_GROUP\Uninstall.lnk" "$INSTDIR\uninst.exe"
  CreateShortCut "$DESKTOP\${PRODUCT_SHORTCUT_NAME}.lnk" "$INSTDIR\${PRODUCT_NAME}.exe"
  !insertmacro MUI_STARTMENU_WRITE_END
SectionEnd

Section -Post
  WriteUninstaller "$INSTDIR\uninst.exe"
  WriteRegStr HKLM "${PRODUCT_DIR_REGKEY}" """$INSTDIR\${PRODUCT_NAME}.exe"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}""DisplayName" "$(^Name)"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}""UninstallString" "$INSTDIR\uninst.exe"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}""DisplayIcon" "$INSTDIR\${PRODUCT_NAME}.exe"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}""DisplayVersion" "${PRODUCT_VERSION}"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}""URLInfoAbout" "${PRODUCT_WEB_SITE}"
  WriteRegStr ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}""Publisher" "${PRODUCT_PUBLISHER}"
SectionEnd

Function un.onUninstSuccess
  HideWindow
  MessageBox MB_ICONINFORMATION|MB_OK "$(^Name) has been successfully removed from your computer."
FunctionEnd

Function un.onInit
!insertmacro MUI_UNGETLANGUAGE
  MessageBox MB_ICONQUESTION|MB_YESNO|MB_DEFBUTTON2 "Are you sure you want to completely remove $(^Name) and all its components?" IDYES +2
  Abort
FunctionEnd

Section Uninstall
  !insertmacro MUI_STARTMENU_GETFOLDER "Application" $ICONS_GROUP
  Delete "$SMPROGRAMS\$ICONS_GROUP\Uninstall.lnk"
  Delete "$SMPROGRAMS\$ICONS_GROUP\${PRODUCT_SHORTCUT_NAME}.lnk"
  RMDir "$SMPROGRAMS\${PRODUCT_NAME}"
  Delete "$DESKTOP\${PRODUCT_SHORTCUT_NAME}.lnk"
  
  RMDir /r "$INSTDIR\*"
  
  DeleteRegKey ${PRODUCT_UNINST_ROOT_KEY} "${PRODUCT_UNINST_KEY}"
  DeleteRegKey HKLM "${PRODUCT_DIR_REGKEY}"
  SetAutoClose true
SectionEnd
