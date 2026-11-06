@echo off
rem FreeDyn 2 Command Prompt (Start menu): this window has the freedyn2 command on its PATH.
rem --home: start in the user profile (the shortcut passes it).
title FreeDyn 2 Command Prompt
set "PATH=%~dp0;%PATH%"
if /i "%~1"=="--home" cd /d "%USERPROFILE%"
freedyn2 --version
echo.
echo   freedyn2 examples              the example models; freedyn2 examples NAME copies one here
echo   freedyn2 run model.yaml        import, build and simulate a model (.yaml or .fds)
echo   freedyn2 view model.yaml       the model and its results in the browser
echo   freedyn2-gui model.fds         the model in the FreeDyn 2 app
echo   freedyn2 doctor                check the installation (compiler, app, MCP)
echo   freedyn2 -h                    all commands
echo.
