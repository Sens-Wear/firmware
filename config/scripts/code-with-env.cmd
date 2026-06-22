@echo off
setlocal

cd /d "%~dp0\..\.."

if exist ".env" (
    for /f "usebackq eol=# tokens=1,* delims==" %%A in (".env") do (
        if not "%%A"=="" set "%%A=%%B"
    )
)

code --new-window .
exit /b %ERRORLEVEL%
