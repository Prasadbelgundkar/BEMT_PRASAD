@echo off
echo ===================================================
echo Tiltrotor BEMT - Automated Setup Script (Windows)
echo ===================================================
echo.
echo Installing required libraries...
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

echo.
echo ===================================================
echo Setup Complete! You can now run the solver.
echo Try running: python run_plots.py
echo ===================================================
pause
