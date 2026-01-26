# Enlistment setup
Enable running .ps1 scripts in PowerShell (once per installed Windows system, not per enlistment)
python3 -m venv .venv (creating the environment - once per enlitment)
.\.venv\bin\Activate.ps1 (once per working session)

# Description
Stress and strain solver for whatever is made out of beams

Math backing the formation of forces and deflections solver is here - https://www.overleaf.com/read/mbwcshybrnzg#36e986
Thus far the solver supports only linearized elasticity case, but it does support statically indeterminate frames (e.g. welded ones).
It does not, at the moment support effects of asembly forces, thermal stresses, and support settling.
