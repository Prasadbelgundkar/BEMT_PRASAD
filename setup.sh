#!/bin/bash
echo "==================================================="
echo "Tiltrotor BEMT - Automated Setup Script (Mac/Linux)"
echo "==================================================="
echo ""
echo "Installing required libraries..."
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt

echo ""
echo "==================================================="
echo "Setup Complete! You can now run the solver."
echo "Try running: python3 run_plots.py"
echo "==================================================="
