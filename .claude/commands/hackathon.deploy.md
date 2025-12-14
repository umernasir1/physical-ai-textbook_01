---
description: Manage deployment - build frontend, deploy to GitHub Pages, test production, and verify deployment status
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

### Purpose
This subagent manages deployment to GitHub Pages, ensuring successful builds and production readiness.

### Capabilities
1. **Build Frontend**: Compile Docusaurus for production
2. **Test Build**: Verify build output and static assets
3. **Deploy**: Push to GitHub Pages via CI/CD
4. **Verify Deployment**: Check live site status
5. **Debug Build Issues**: Fix compilation errors and deployment problems

### Execution Steps

1. **Understand Request**:
   - Parse action: build | test | deploy | verify | debug
   - If no action, check deployment status

2. **Build Frontend**:
   - Navigate to `frontend/` directory
   - Run pre-build checks:
     - Verify `docusaurus.config.js` settings:
       - url: `https://umernasir1.github.io`
       - baseUrl: `/physical-ai-textbook/`
       - organizationName: `umernasir1`
       - projectName: `physical-ai-textbook`
     - Check all `.md` files for MDX syntax errors
     - Verify React components compile
   - Execute build:
     - Run `npm run build`
     - Monitor build output for errors
     - Check build time and bundle size
   - Verify build artifacts:
     - `build/` directory exists
     - `index.html` generated
     - Static assets bundled
     - No missing files
   - Report: Build success/failure, errors, warnings, bundle size

3. **Test Build**:
   - Serve production build locally:
     - Run `npm run serve` (or `npx http-server build/`)
     - Test at `http://localhost:3000`
   - Verify functionality:
     - All pages load correctly
     - Navigation works
     - Chatbot widget appears
     - Translation button present
     - Code highlighting works
     - No console errors
   - Check performance:
     - Page load speed
     - Asset loading
     - JavaScript errors
   - Report: Test results, issues found

4. **Deploy to GitHub Pages**:
   - Verify GitHub Actions workflow:
     - Check `.github/workflows/deploy.yml` exists
     - Verify workflow triggers: push to main/master
     - Confirm Node.js version: 20
   - Trigger deployment:
     - Option 1: Push to main branch
       ```bash
       git status
       git add .
       git commit -m "Deploy to GitHub Pages"
       git push origin main
       ```
     - Option 2: Manual workflow dispatch
   - Monitor GitHub Actions:
     - Check workflow run status
     - View build logs
     - Verify deployment step completes
   - Report: Deployment status, workflow URL, errors if any

5. **Verify Deployment**:
   - Check live site:
     - URL: `https://umernasir1.github.io/physical-ai-textbook/`
     - Verify page loads
     - Test navigation
     - Check all modules accessible
     - Verify chatbot works (may need backend URL update)
   - Test critical features:
     - All 4 modules visible
     - All chapters load
     - Code blocks render
     - Search works
     - No 404 errors
   - Check GitHub Pages settings:
     - Repository settings > Pages
     - Source: gh-pages branch
     - Custom domain (if any)
   - Report: Site status, URL, issues, feature checklist

6. **Debug Build/Deploy Issues**:
   - Common problems:
     - MDX compilation errors (check for `<` in tables)
     - Missing dependencies (`npm install`)
     - Build timeout (check bundle size)
     - 404 on deployment (baseUrl mismatch)
     - Assets not loading (path issues)
     - GitHub Actions failing (check logs)
   - For each issue:
     - Diagnose from build logs
     - Apply fix
     - Retry build
     - Verify fix worked
   - Report: Issues found, fixes applied, status

7. **Report Summary**:
   - Build status: Success | Failed | In Progress
   - Deployment status: Live | Pending | Failed
   - Live URL: `https://umernasir1.github.io/physical-ai-textbook/`
   - Last deploy: timestamp
   - Issues: N found, M fixed
   - Next actions if any

### Usage Examples

```bash
# Build frontend
/hackathon.deploy build

# Test production build
/hackathon.deploy test

# Deploy to GitHub Pages
/hackathon.deploy

# Verify live deployment
/hackathon.deploy verify

# Debug deployment issues
/hackathon.deploy debug
```

### Success Criteria
- Frontend builds without errors
- All features work in production build
- GitHub Actions workflow succeeds
- Live site accessible at correct URL
- All modules and chapters load
- No 404 or broken links
