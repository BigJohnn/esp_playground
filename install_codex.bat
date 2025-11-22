@echo off
setlocal

rem === ���� ===
set "URL=http://8.152.4.210:5244/d/data/scripts/install_codex.ps1"
rem ���浽��ǰ .bat ����Ŀ¼
set "DEST=%~dp0install_codex.ps1"

echo [INFO] Target: %DEST%
if exist "%DEST%" del /f /q "%DEST%" >nul 2>&1

echo [INFO] Downloading (curl if available)...
where curl >nul 2>&1
IF %ERRORLEVEL% EQU 0 curl --silent --show-error --fail -L "%URL%" -o "%DEST%"

IF NOT EXIST "%DEST%" powershell -NoLogo -NoProfile -ExecutionPolicy Bypass -Command "try{$ProgressPreference='SilentlyContinue';Invoke-WebRequest -Uri '%URL%' -OutFile '%DEST%' -UseBasicParsing}catch{exit 1}"

IF NOT EXIST "%DEST%" (
  echo [ERROR] Download failed. Check the URL or network.
  exit /b 1
)

echo [INFO] Unblocking downloaded file...
powershell -NoLogo -NoProfile -ExecutionPolicy Bypass -Command "try{Unblock-File -LiteralPath '%DEST%' -ErrorAction SilentlyContinue}catch{}"

for %%A in ("%DEST%") do set "SIZE=%%~zA"
echo [INFO] Downloaded (%SIZE% bytes). Running...

powershell -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%DEST%" %*
set "code=%ERRORLEVEL%"
echo [INFO] Finished with exit code %code%
exit /b %code%
