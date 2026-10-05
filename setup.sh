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
echo "Run the tests:      python3 -m pytest"
echo "Milestone 2 figures: python3 scripts/m2/run_all_m2.py"
echo "==================================================="
