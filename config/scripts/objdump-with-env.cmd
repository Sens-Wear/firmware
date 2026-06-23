@echo off
setlocal

cd /d "%~dp0\..\.."

if exist ".env" (
    for /f "usebackq eol=# tokens=1,* delims==" %%A in (".env") do (
        if not "%%A"=="" set "%%A=%%B"
    )
)

if "%ZEPHYR_OBJDUMP%"=="" if not "%ZEPHYR_GDB%"=="" (
    set "ZEPHYR_OBJDUMP=%ZEPHYR_GDB:-gdb=-objdump%"
)

if "%ZEPHYR_OBJDUMP%"=="" (
    echo ZEPHYR_OBJDUMP was not found. Set ZEPHYR_GDB or ZEPHYR_OBJDUMP in .env. 1>&2
    exit /b 1
)

if not exist "%ZEPHYR_OBJDUMP%" (
    echo ZEPHYR_OBJDUMP does not point to an executable: %ZEPHYR_OBJDUMP% 1>&2
    exit /b 1
)

"%ZEPHYR_OBJDUMP%" %*
exit /b %ERRORLEVEL%
