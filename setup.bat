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
echo Run the tests:       python -m pytest
echo Milestone 2 figures: python scripts/m2/run_all_m2.py
echo ===================================================
pause
