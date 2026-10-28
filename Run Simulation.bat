@echo off
cd /d "%~dp0"
".venv\Scripts\python.exe" turtlebot.py
if errorlevel 1 pause
