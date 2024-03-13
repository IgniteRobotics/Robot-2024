@echo OFF
:: ---------------------------------------------------------------------
:: Script      : fetchMyfiles.bat
:: Description : SCP files from a remote machine.
::               Requires the OpenSSH client Windows Capability to be installed (Windows Server 2019)
:: ---------------------------------------------------------------------
::
:: Variables
::
set rootdir="%~dp0"
set logdir=%rootdir%\wpilogs
set logfile=%logdir%\myfiles.log
set backupdir=%rootdir%\riobackups
set remotehost=10.68.29.2
set remoteuser=lvuser
set logpath="~/logs/*"
set ntpath="~/"
set ntfile="networktables.json"
set timestamp=%DATE:/=-%_%TIME::=-%
set timestamp=%timestamp: =%
::
:: Main
::
echo %DATE% : %TIME% : INFO : Starting riodownload.bat >> %logfile%
echo Timestamp: %timestamp%
if not exist %logdir% mkdir %logdir%
if not exist %backupdir% mkdir %backupdir%
scp.exe %remoteuser%@%remotehost%:%logpath% %logdir% >> %logfile%
scp.exe %remoteuser%@%remotehost%:%logpath% %backupdir%\%ntfile%.%timestamp%
if %ERRORLEVEL% equ 0 (
		echo %DATE% : %TIME% : INFO : Fetched logs >> %logfile%
		) else (
		echo %DATE% : %TIME% : ERROR : Failed to fetch logs >> %logfile%
		)

