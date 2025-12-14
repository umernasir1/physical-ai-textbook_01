@echo off
echo ============================================
echo   PHYSICAL AI TEXTBOOK - DEPLOYMENT SCRIPT
echo ============================================
echo.

REM Get user input
set /p GITHUB_USERNAME="Enter your GitHub username: "
set /p REPO_NAME="Enter repository name (default: physical-ai-textbook): "

REM Set default if empty
if "%REPO_NAME%"=="" set REPO_NAME=physical-ai-textbook

echo.
echo --------------------------------------------
echo Configuration:
echo   GitHub Username: %GITHUB_USERNAME%
echo   Repository Name: %REPO_NAME%
echo   Repository URL: https://github.com/%GITHUB_USERNAME%/%REPO_NAME%
echo --------------------------------------------
echo.

pause

echo.
echo Step 1: Adding GitHub remote...
git remote remove origin 2>nul
git remote add origin https://github.com/%GITHUB_USERNAME%/%REPO_NAME%.git
if %errorlevel% neq 0 (
    echo ERROR: Failed to add remote. Please check the repository exists.
    pause
    exit /b 1
)
echo SUCCESS: Remote added!

echo.
echo Step 2: Pushing to GitHub...
git push -u origin master
if %errorlevel% neq 0 (
    echo.
    echo ERROR: Failed to push. This might be because:
    echo   1. Repository doesn't exist on GitHub yet
    echo   2. You need to authenticate
    echo   3. Branch name might be 'main' instead of 'master'
    echo.
    echo Please create the repository on GitHub first:
    echo   https://github.com/new
    echo.
    echo Then try running this script again.
    pause
    exit /b 1
)

echo.
echo ============================================
echo   SUCCESS! Code pushed to GitHub!
echo ============================================
echo.
echo Next steps:
echo   1. Go to: https://github.com/%GITHUB_USERNAME%/%REPO_NAME%/settings/pages
echo   2. Enable GitHub Pages:
echo      - Source: Deploy from a branch
echo      - Branch: gh-pages
echo      - Folder: / (root)
echo   3. Wait 2-3 minutes for deployment
echo   4. Visit: https://%GITHUB_USERNAME%.github.io/%REPO_NAME%/
echo.
echo IMPORTANT: Update docusaurus.config.js with your GitHub details!
echo.

pause
