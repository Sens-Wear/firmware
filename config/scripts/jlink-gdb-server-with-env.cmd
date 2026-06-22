@echo off
setlocal

cd /d "%~dp0\..\.."

if exist ".env" (
    for /f "usebackq eol=# tokens=1,* delims==" %%A in (".env") do (
        if not "%%A"=="" set "%%A=%%B"
    )
)

if "%JLINK_GDB_SERVER%"=="" (
    for %%E in (JLinkGDBServerCLExe.exe JLinkGDBServerCL.exe JLinkGDBServer.exe) do (
        where %%E >nul 2>&1
        if not errorlevel 1 if "%JLINK_GDB_SERVER%"=="" set "JLINK_GDB_SERVER=%%E"
    )
)

if "%JLINK_GDB_SERVER%"=="" (
    echo J-Link GDB server was not found. Set JLINK_GDB_SERVER in .env. 1>&2
    exit /b 1
)

where "%JLINK_GDB_SERVER%" >nul 2>&1
if errorlevel 1 if not exist "%JLINK_GDB_SERVER%" (
    echo JLINK_GDB_SERVER does not point to an executable: %JLINK_GDB_SERVER% 1>&2
    exit /b 1
)

"%JLINK_GDB_SERVER%" %*
exit /b %ERRORLEVEL%
