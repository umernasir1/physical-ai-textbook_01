@echo off
echo ============================================
echo   DEPLOYING FOR: umernasir1
echo   Repository: physical-ai-textbook
echo ============================================
echo.

echo Step 1: Removing any existing remote...
git remote remove origin 2>nul

echo Step 2: Adding GitHub remote...
git remote add origin https://github.com/umernasir1/physical-ai-textbook.git
if %errorlevel% neq 0 (
    echo ERROR: Failed to add remote
    pause
    exit /b 1
)
echo SUCCESS: Remote added!

echo.
echo Step 3: Checking current branch...
git branch
echo.

echo Step 4: Ready to push!
echo.
echo IMPORTANT: When prompted, enter:
echo   Username: umernasir1
echo   Password: [YOUR_PERSONAL_ACCESS_TOKEN]
echo.
echo If you don't have a token yet:
echo   1. Go to: https://github.com/settings/tokens
echo   2. Generate new token (classic)
echo   3. Check 'repo' and 'workflow' scopes
echo   4. Copy the token and use it as password
echo.

pause

echo.
echo Pushing to GitHub...
git push -u origin setup-backend
if %errorlevel% neq 0 (
    echo.
    echo Push failed. Trying with 'master' branch...
    git push -u origin master
    if %errorlevel% neq 0 (
        echo.
        echo ERROR: Push failed. Please check:
        echo   1. Repository exists at: https://github.com/umernasir1/physical-ai-textbook
        echo   2. Repository is PUBLIC
        echo   3. You used correct credentials
        pause
        exit /b 1
    )
)

echo.
echo ============================================
echo   SUCCESS! Code pushed to GitHub!
echo ============================================
echo.
echo NEXT STEPS:
echo.
echo 1. Enable GitHub Pages:
echo    https://github.com/umernasir1/physical-ai-textbook/settings/pages
echo    - Source: Deploy from a branch
echo    - Branch: gh-pages
echo    - Click Save
echo.
echo 2. Monitor deployment:
echo    https://github.com/umernasir1/physical-ai-textbook/actions
echo.
echo 3. Your site will be live at:
echo    https://umernasir1.github.io/physical-ai-textbook/
echo.
echo 4. Run UPDATE_UMERNASIR1.bat to update configuration
echo.

pause
