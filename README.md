# Enlistment setup
Once per installed Windows system, not per enlistment
1. Enable running .ps1 scripts in PowerShell
2. Install cmake from oficial cmake website
3. Install "Build Tools for Visual Studio" from the official Visual Studio downloads page. During installation check the "Desktop development with C++" workload.
Once per enlitment
1. python3 -m venv .venv
2. .\.venv\scripts\Activate.ps1
3. pip install numpy
Once per working session
1. .\.venv\scripts\Activate.ps1

# Description
Stress and strain solver for whatever is made out of beams

Math backing the formation of forces and deflections solver is here - https://www.overleaf.com/read/mbwcshybrnzg#36e986
Thus far the solver supports only linearized elasticity case, but it does support statically indeterminate frames (e.g. welded ones).
It does not, at the moment support effects of asembly forces, thermal stresses, and support settling.
