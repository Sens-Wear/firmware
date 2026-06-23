@echo off
setlocal

cd /d "%~dp0\..\.."

if exist ".env" (
    for /f "usebackq eol=# tokens=1,* delims==" %%A in (".env") do (
        if not "%%A"=="" set "%%A=%%B"
    )
)

if "%ZEPHYR_NM%"=="" if not "%ZEPHYR_GDB%"=="" (
    set "ZEPHYR_NM=%ZEPHYR_GDB:-gdb=-nm%"
)

if "%ZEPHYR_NM%"=="" (
    echo ZEPHYR_NM was not found. Set ZEPHYR_GDB or ZEPHYR_NM in .env. 1>&2
    exit /b 1
)

if not exist "%ZEPHYR_NM%" (
    echo ZEPHYR_NM does not point to an executable: %ZEPHYR_NM% 1>&2
    exit /b 1
)

"%ZEPHYR_NM%" %*
exit /b %ERRORLEVEL%
