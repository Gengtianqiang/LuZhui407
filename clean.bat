@echo off
echo ============================================
echo   Cleaning TH_HAL build files...
echo ============================================
echo.

set "PROJECT_DIR=%~dp0"
cd /d "%PROJECT_DIR%"

REM === 1. MDK-ARM\HAL build artifacts ===
if exist "MDK-ARM\HAL\" (
    echo [1/5] Cleaning MDK-ARM\HAL\ ...
    del "MDK-ARM\HAL\*.o"   2>nul
    del "MDK-ARM\HAL\*.d"   2>nul
    del "MDK-ARM\HAL\*.crf" 2>nul
    del "MDK-ARM\HAL\*.htm" 2>nul
    del "MDK-ARM\HAL\*.dep" 2>nul
    del "MDK-ARM\HAL\*.iex" 2>nul
    del "MDK-ARM\HAL\*.axf" 2>nul
    del "MDK-ARM\HAL\*.hex" 2>nul
    del "MDK-ARM\HAL\*.lst" 2>nul
    del "MDK-ARM\HAL\*.map" 2>nul
    del "MDK-ARM\HAL\*.Bak" 2>nul
    del "MDK-ARM\HAL\*.ini" 2>nul
    echo    Done.
)

REM === 2. build\ directory ===
if exist "build\" (
    echo [2/5] Deleting build\ ...
    rmdir /s /q "build" 2>nul
    echo    Done.
)

REM === 3. MDK-ARM\RTE\ directory ===
if exist "MDK-ARM\RTE\" (
    echo [3/5] Deleting MDK-ARM\RTE\ ...
    rmdir /s /q "MDK-ARM\RTE" 2>nul
    echo    Done.
)

REM === 4. MDK-ARM\DebugConfig\ directory ===
if exist "MDK-ARM\DebugConfig\" (
    echo [4/5] Deleting MDK-ARM\DebugConfig\ ...
    rmdir /s /q "MDK-ARM\DebugConfig" 2>nul
    echo    Done.
)

REM === 5. RTE_Components.h ===
if exist "RTE_Components.h" (
    echo [5/5] Deleting RTE_Components.h ...
    del "RTE_Components.h" 2>nul
    echo    Done.
)

echo.
echo ============================================
echo   Clean complete!
echo ============================================
pause
