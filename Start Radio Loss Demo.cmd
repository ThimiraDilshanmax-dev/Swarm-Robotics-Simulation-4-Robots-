@echo off
cd /d "%~dp0"
python run_simulation.py --scenario radio_loss
if errorlevel 1 pause

