# 🚀 Deployment Guide - Physical AI Textbook

This guide will help you deploy both the frontend (Docusaurus) and backend (FastAPI) to production.

## 📋 Prerequisites

- GitHub account
- Git installed locally
- Railway, Render, or Vercel account (for backend)
- All environment variables ready

---

## 🎯 Deployment Strategy

| Component | Platform | URL Pattern |
|-----------|----------|-------------|
| **Frontend** | GitHub Pages | `https://YOUR_USERNAME.github.io/physical-ai-textbook/` |
| **Backend** | Railway/Render | `https://YOUR_APP.railway.app` or `https://YOUR_APP.onrender.com` |

---

## Part 1: Deploy Backend (FastAPI)

### Option A: Deploy to Railway (Recommended - Free Tier)

1. **Create Railway Account**
   - Go to: https://railway.app
   - Sign up with GitHub

2. **Create New Project**
   - Click "New Project"
   - Select "Deploy from GitHub repo"
   - Connect your GitHub account
   - Select your repository

3. **Configure Backend**
   - Railway will auto-detect Python
   - Set root directory: `backend`
   - Add environment variables:

   ```env
   GROQ_API_KEY=your_groq_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   NEON_DATABASE_URL=your_neon_db_url
   SECRET_KEY=your_secret_key_here
   ACCESS_TOKEN_EXPIRE_MINUTES=30
   PORT=8000
   ```

4. **Deploy**
   - Click "Deploy"
   - Railway will build and deploy automatically
   - Get your backend URL: `https://YOUR_APP.railway.app`

### Option B: Deploy to Render (Alternative Free Tier)

1. **Create Render Account**
   - Go to: https://render.com
   - Sign up with GitHub

2. **Create New Web Service**
   - Click "New +" → "Web Service"
   - Connect your GitHub repo
   - Select the repository

3. **Configure Service**
   ```yaml
   Name: physical-ai-backend
   Environment: Python 3
   Build Command: pip install -r requirements.txt
   Start Command: uvicorn src.main:app --host 0.0.0.0 --port $PORT
   ```

4. **Set Environment Variables**
   - Add all the same variables as Railway above

5. **Deploy**
   - Click "Create Web Service"
   - Get your URL: `https://YOUR_APP.onrender.com`

### Option C: Deploy to Vercel (Serverless)

1. **Install Vercel CLI**
   ```bash
   npm install -g vercel
   ```

2. **Deploy**
   ```bash
   cd backend
   vercel
   ```

3. **Configure**
   - Set environment variables in Vercel dashboard
   - Vercel will provide a URL: `https://YOUR_APP.vercel.app`

---

## Part 2: Deploy Frontend (Docusaurus to GitHub Pages)

### Step 1: Create GitHub Repository

1. **Create New Repository**
   - Go to: https://github.com/new
   - Repository name: `physical-ai-textbook`
   - Make it Public
   - Don't initialize with README (we already have one)
   - Click "Create repository"

2. **Connect Local Repository**
   ```bash
   # Navigate to your project
   cd "D:\PIAIC Batch 76\Hackaton"

   # Initialize git (if not already)
   git init

   # Add remote
   git remote add origin https://github.com/YOUR_USERNAME/physical-ai-textbook.git

   # Verify
   git remote -v
   ```

### Step 2: Update Frontend Configuration

1. **Update `frontend/docusaurus.config.js`**

   Change lines 23 and 35-36 to match YOUR GitHub username:
   ```javascript
   url: 'https://YOUR_GITHUB_USERNAME.github.io',
   organizationName: 'YOUR_GITHUB_USERNAME',
   projectName: 'physical-ai-textbook',
   ```

2. **Set Production Backend URL**

   Update line 30 to use your deployed backend:
   ```javascript
   customFields: {
     BACKEND_API_URL: process.env.BACKEND_API_URL || 'https://YOUR_BACKEND_URL/api/v1',
   },
   ```

### Step 3: Enable GitHub Pages

1. **Push to GitHub**
   ```bash
   # Add all files
   git add .

   # Commit changes
   git commit -m "Initial deployment setup"

   # Push to main branch
   git push -u origin setup-backend:main
   ```

2. **Enable GitHub Pages**
   - Go to your GitHub repository
   - Click "Settings" → "Pages"
   - Source: Deploy from a branch
   - Branch: `gh-pages` / `root`
   - Click "Save"

3. **Wait for Deployment**
   - Go to "Actions" tab
   - Watch the deployment workflow run
   - Once complete (✅), visit your site!

---

## 🔧 Update CORS Settings

After deploying backend, update `backend/src/main.py`:

```python
origins = [
    "http://localhost",
    "http://localhost:3000",
    "https://YOUR_GITHUB_USERNAME.github.io",  # Add this
]
```

Redeploy the backend after this change.

---

## ✅ Verification Checklist

### Backend Deployed ✅
- [ ] Visit: `https://YOUR_BACKEND_URL/api/v1/`
- [ ] Should return: `{"message": "Welcome to the FastAPI Backend!"}`
- [ ] Check: `https://YOUR_BACKEND_URL/docs`
- [ ] API documentation should load

### Frontend Deployed ✅
- [ ] Visit: `https://YOUR_GITHUB_USERNAME.github.io/physical-ai-textbook/`
- [ ] Homepage loads correctly
- [ ] Visit: `https://YOUR_GITHUB_USERNAME.github.io/physical-ai-textbook/auth`
- [ ] Auth page loads without errors

### Authentication Working ✅
- [ ] Sign up creates account
- [ ] Login works
- [ ] Redirect to homepage succeeds
- [ ] Token stored in localStorage

### Features Working ✅
- [ ] Chatbot responds to queries
- [ ] Translation works
- [ ] Documentation pages load
- [ ] No CORS errors in console

---

## 🐛 Troubleshooting

### Backend Issues

**Problem**: Backend not starting
- Check logs in Railway/Render dashboard
- Verify all environment variables are set
- Check Python version (should be 3.11+)

**Problem**: CORS errors
- Make sure frontend URL is in `origins` list
- Redeploy backend after updating CORS

### Frontend Issues

**Problem**: 404 on GitHub Pages
- Check `baseUrl` in docusaurus.config.js
- Verify repository name matches projectName
- Wait 5-10 minutes after first deployment

**Problem**: Auth page crashes
- Check browser console for errors
- Verify AuthProvider is in Root.js
- Check backend URL is correct

---

## 🎉 You're Done!

Your project is now live at:
- **Frontend**: `https://YOUR_GITHUB_USERNAME.github.io/physical-ai-textbook/`
- **Backend**: `https://YOUR_BACKEND_URL/`

Share your project and enjoy! 🚀

---

## 📱 Next Steps

1. **Custom Domain** (Optional)
   - Buy a domain (e.g., from Namecheap)
   - Configure DNS
   - Update GitHub Pages settings

2. **Database Migration**
   - Currently using in-memory storage
   - Add proper Neon Postgres integration
   - Implement user persistence

3. **Monitoring**
   - Set up error tracking (Sentry)
   - Add analytics (Google Analytics)
   - Monitor API usage

4. **Performance**
   - Enable CDN caching
   - Optimize images
   - Add service worker for PWA
