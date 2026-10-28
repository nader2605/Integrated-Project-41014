@echo off
cd /d "%~dp0"
".venv\Scripts\python.exe" demo.py
if errorlevel 1 pause
