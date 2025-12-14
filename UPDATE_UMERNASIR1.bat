@echo off
echo ============================================
echo   UPDATING CONFIGURATION FOR: umernasir1
echo ============================================
echo.

echo Updating frontend/docusaurus.config.js...

powershell -Command "(Get-Content 'frontend/docusaurus.config.js' -Raw) -replace \"url: 'https://.*?'\", \"url: 'https://umernasir1.github.io'\" -replace \"baseUrl: '/.*?/'\", \"baseUrl: '/physical-ai-textbook/'\" -replace \"organizationName: '.*?'\", \"organizationName: 'umernasir1'\" -replace \"projectName: '.*?'\", \"projectName: 'physical-ai-textbook'\" | Set-Content 'frontend/docusaurus.config.js' -NoNewline"

if %errorlevel% neq 0 (
    echo ERROR: Failed to update configuration
    pause
    exit /b 1
)

echo SUCCESS: Configuration updated!
echo.
echo Changes made:
echo   url: 'https://umernasir1.github.io'
echo   baseUrl: '/physical-ai-textbook/'
echo   organizationName: 'umernasir1'
echo   projectName: 'physical-ai-textbook'
echo.

echo Committing changes...
git add frontend/docusaurus.config.js
git commit -m "chore: Update GitHub Pages configuration for umernasir1"

echo.
echo Pushing to GitHub...
git push origin setup-backend
if %errorlevel% neq 0 (
    git push origin master
)

echo.
echo ============================================
echo   CONFIGURATION UPDATED AND PUSHED!
echo ============================================
echo.
echo Wait 2-3 minutes for redeployment, then visit:
echo https://umernasir1.github.io/physical-ai-textbook/
echo.

pause
