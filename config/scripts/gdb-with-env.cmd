@echo off
setlocal

cd /d "%~dp0\..\.."

if exist ".env" (
    for /f "usebackq eol=# tokens=1,* delims==" %%A in (".env") do (
        if not "%%A"=="" set "%%A=%%B"
    )
)

if "%ZEPHYR_GDB%"=="" (
    echo ZEPHYR_GDB is not set. Copy .env.example to .env and set ZEPHYR_GDB. 1>&2
    exit /b 1
)

if not exist "%ZEPHYR_GDB%" (
    echo ZEPHYR_GDB does not point to an executable: %ZEPHYR_GDB% 1>&2
    exit /b 1
)

"%ZEPHYR_GDB%" %*
exit /b %ERRORLEVEL%
