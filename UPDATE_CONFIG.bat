@echo off
echo ============================================
echo   UPDATE DOCUSAURUS CONFIGURATION
echo ============================================
echo.

set /p GITHUB_USERNAME="Enter your GitHub username: "
set /p REPO_NAME="Enter repository name (default: physical-ai-textbook): "

if "%REPO_NAME%"=="" set REPO_NAME=physical-ai-textbook

echo.
echo This will update frontend/docusaurus.config.js with:
echo   url: 'https://%GITHUB_USERNAME%.github.io'
echo   baseUrl: '/%REPO_NAME%/'
echo   organizationName: '%GITHUB_USERNAME%'
echo   projectName: '%REPO_NAME%'
echo.

pause

REM Create a temporary PowerShell script to update the config
echo $content = Get-Content 'frontend/docusaurus.config.js' -Raw > update_config.ps1
echo $content = $content -replace "url: 'https://.*?'", "url: 'https://%GITHUB_USERNAME%.github.io'" >> update_config.ps1
echo $content = $content -replace "baseUrl: '/.*?/'", "baseUrl: '/%REPO_NAME%/'" >> update_config.ps1
echo $content = $content -replace "organizationName: '.*?'", "organizationName: '%GITHUB_USERNAME%'" >> update_config.ps1
echo $content = $content -replace "projectName: '.*?'", "projectName: '%REPO_NAME%'" >> update_config.ps1
echo $content ^| Set-Content 'frontend/docusaurus.config.js' -NoNewline >> update_config.ps1

powershell -ExecutionPolicy Bypass -File update_config.ps1
del update_config.ps1

echo.
echo Configuration updated!
echo.
echo Now commit and push:
echo   git add frontend/docusaurus.config.js
echo   git commit -m "chore: Update GitHub Pages configuration"
echo   git push origin master
echo.

pause
