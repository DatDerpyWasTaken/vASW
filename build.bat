@echo off
setlocal
cd /d "%~dp0"

where python >nul 2>nul
if errorlevel 1 (
    echo Python was not found on PATH.
    exit /b 1
)

python -m PyInstaller --version >nul 2>nul
if errorlevel 1 (
    echo PyInstaller is not installed for this Python.
    echo Install it with: python -m pip install pyinstaller
    exit /b 1
)

python -m PyInstaller ^
  --noconfirm ^
  --clean ^
  -D ^
  --name vASW ^
  --contents-directory . ^
  --add-data "config.json;." ^
  --add-data "gaist_models.json;." ^
  --add-data "menu_theme;." ^
  --add-data "assets;assets" ^
  --add-data "Tiled.tmx;." ^
  --add-data "Tileset.png;." ^
  --add-data "SimConnect;SimConnect" ^
  --add-data "aircraft_position.json;." ^
  --add-data "docs;docs" ^
  --hidden-import scipy.signal ^
  --hidden-import matplotlib.backends.backend_agg ^
  vASW.py

if errorlevel 1 (
    echo Build failed.
    exit /b 1
)

echo Build complete: dist\vASW\vASW.exe
