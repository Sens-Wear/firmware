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

rem OpenOCD does not service the GDB socket until the launch commands that flash
rem and verify the image have finished, which takes longer than GDB's 2 s
rem default. Without a longer timeout GDB retransmits qSupported, OpenOCD
rem answers every copy, and the replies desync ("Remote replied unexpectedly to
rem 'vMustReplyEmpty'").
"%ZEPHYR_GDB%" -iex "set remotetimeout 60" %*
exit /b %ERRORLEVEL%
