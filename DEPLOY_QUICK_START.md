# ⚡ Quick Deployment Guide - 5 Minutes

## 🎯 What You'll Deploy

- **Frontend**: GitHub Pages (Free, Auto-deploys on push)
- **Backend**: Railway or Render (Free tier available)

---

## Step 1: Deploy Backend First (5 minutes)

### Using Railway (Easiest)

1. Go to https://railway.app and sign in with GitHub

2. Click "New Project" → "Deploy from GitHub repo"

3. Connect and select your repository

4. Configure:
   - **Root Directory**: `backend`
   - **Start Command**: (Auto-detected)

5. Add Environment Variables:
   ```
   GROQ_API_KEY=<your-groq-key>
   QDRANT_URL=<your-qdrant-url>
   QDRANT_API_KEY=<your-qdrant-key>
   NEON_DATABASE_URL=<your-neon-db-url>
   SECRET_KEY=<generate-random-secret>
   ACCESS_TOKEN_EXPIRE_MINUTES=30
   ```

6. Deploy! 🚀

7. Copy your backend URL: `https://your-app.railway.app`

---

## Step 2: Update Frontend Config (2 minutes)

### Update Docusaurus Config

Edit `frontend/docusaurus.config.js`:

```javascript
// Line 23: Update with your GitHub username
url: 'https://YOUR_GITHUB_USERNAME.github.io',

// Line 30: Update with your Railway backend URL
customFields: {
  BACKEND_API_URL: 'https://your-app.railway.app/api/v1',
},

// Lines 35-36: Update with your GitHub username
organizationName: 'YOUR_GITHUB_USERNAME',
projectName: 'physical-ai-textbook',
```

### Update Backend CORS

Edit `backend/src/main.py`:

```python
origins = [
    "http://localhost",
    "http://localhost:3000",
    "https://YOUR_GITHUB_USERNAME.github.io",  # Add this line
]
```

Commit and push to redeploy backend.

---

## Step 3: Deploy to GitHub Pages (3 minutes)

### First Time Setup

```bash
# 1. Create repository on GitHub
# Go to https://github.com/new
# Name: physical-ai-textbook
# Make it Public
# Don't initialize with anything

# 2. Connect local repo
cd "D:\PIAIC Batch 76\Hackaton"
git init
git remote add origin https://github.com/YOUR_USERNAME/physical-ai-textbook.git

# 3. Commit all changes
git add .
git commit -m "🚀 Initial deployment"

# 4. Push to main
git push -u origin setup-backend:main
```

### Enable GitHub Pages

1. Go to your repo → Settings → Pages
2. Source: "Deploy from a branch"
3. Branch: `gh-pages` / `root`
4. Save

### Wait for Deployment

- Go to "Actions" tab
- Watch the workflow run (takes ~5 minutes)
- Once complete, visit: `https://YOUR_USERNAME.github.io/physical-ai-textbook/`

---

## ✅ Verification

### Test Backend
```bash
curl https://your-app.railway.app/api/v1/
```
Should return: `{"message": "Welcome to the FastAPI Backend!"}`

### Test Frontend
Visit: `https://YOUR_USERNAME.github.io/physical-ai-textbook/`

### Test Auth
1. Go to `/auth` page
2. Sign up
3. Login
4. Should redirect to homepage ✅

---

## 🎉 Done!

Your app is live! Share the link:
`https://YOUR_USERNAME.github.io/physical-ai-textbook/`

---

## Need Help?

See full guide: `DEPLOYMENT_GUIDE.md`
