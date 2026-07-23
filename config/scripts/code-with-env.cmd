@echo off
setlocal

cd /d "%~dp0\..\.."

if exist ".env" (
    for /f "usebackq eol=# tokens=1,* delims==" %%A in (".env") do (
        if not "%%A"=="" set "%%A=%%B"
    )
)

if not "%NCS_TOOLCHAIN_ROOT%"=="" (
    set "PATH=%NCS_TOOLCHAIN_ROOT%\opt\zephyr-sdk\gnu\arm-zephyr-eabi\bin;%NCS_TOOLCHAIN_ROOT%\opt\bin;%NCS_TOOLCHAIN_ROOT%\opt\bin\Scripts;%NCS_TOOLCHAIN_ROOT%\bin;%PATH%"
)

code --new-window .
exit /b %ERRORLEVEL%
