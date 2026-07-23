@echo off
setlocal

cd /d "%~dp0\..\.."

if exist ".env" (
    for /f "usebackq eol=# tokens=1,* delims==" %%A in (".env") do (
        if not "%%A"=="" set "%%A=%%B"
    )
)

if "%OPENOCD%"=="" (
    echo OPENOCD is not set. Copy .env.example to .env and set OPENOCD. 1>&2
    exit /b 1
)

if not exist "%OPENOCD%" (
    echo OPENOCD does not point to an executable: %OPENOCD% 1>&2
    exit /b 1
)

if not "%OPENOCD_SCRIPTS%"=="" (
    "%OPENOCD%" -s "%OPENOCD_SCRIPTS%" %*
) else (
    "%OPENOCD%" %*
)

exit /b %ERRORLEVEL%
