# 🚀 Deploy to Vercel + GitHub Pages

## Quick Deployment Guide for umernasir1

This guide will deploy:
- **Backend** → Vercel (Serverless FastAPI)
- **Frontend** → GitHub Pages (Static Site)

---

## ✅ Prerequisites

- [x] GitHub account: `umernasir1` ✓
- [ ] Vercel account (sign up with GitHub)
- [ ] All API keys ready

### Required API Keys

You'll need these for Vercel environment variables:

```env
GROQ_API_KEY=your_groq_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_database_url
SECRET_KEY=generate_random_32_char_string
ACCESS_TOKEN_EXPIRE_MINUTES=30
```

Generate SECRET_KEY:
```bash
openssl rand -hex 32
```

---

## Part 1: Deploy Backend to Vercel (5 minutes)

### Option A: Using Vercel Dashboard (Easiest)

1. **Sign Up for Vercel**
   - Go to: https://vercel.com
   - Click "Sign Up"
   - Choose "Continue with GitHub"
   - Authorize Vercel

2. **Import Your Repository**
   - Click "Add New..." → "Project"
   - Import from GitHub
   - Select: `umernasir1/physical-ai-textbook`
   - Click "Import"

3. **Configure Project**
   ```
   Framework Preset: Other
   Root Directory: backend
   Build Command: (leave empty)
   Output Directory: (leave empty)
   Install Command: pip install -r requirements.txt
   ```

4. **Add Environment Variables**

   Click "Environment Variables" and add:

   | Name | Value |
   |------|-------|
   | `GROQ_API_KEY` | Your Groq API key |
   | `QDRANT_URL` | Your Qdrant URL |
   | `QDRANT_API_KEY` | Your Qdrant API key |
   | `NEON_DATABASE_URL` | Your Neon database URL |
   | `SECRET_KEY` | Random 32-char hex string |
   | `ACCESS_TOKEN_EXPIRE_MINUTES` | `30` |

5. **Deploy**
   - Click "Deploy"
   - Wait 2-3 minutes
   - Get your URL: `https://physical-ai-textbook-backend.vercel.app`

### Option B: Using Vercel CLI

```bash
# Install Vercel CLI
npm install -g vercel

# Navigate to backend
cd backend

# Login to Vercel
vercel login

# Deploy
vercel

# Follow prompts:
# Set up and deploy? Yes
# Which scope? Your account
# Link to existing project? No
# Project name? physical-ai-textbook-backend
# Directory? ./
# Override settings? No

# Add environment variables
vercel env add GROQ_API_KEY
vercel env add QDRANT_URL
vercel env add QDRANT_API_KEY
vercel env add NEON_DATABASE_URL
vercel env add SECRET_KEY
vercel env add ACCESS_TOKEN_EXPIRE_MINUTES

# Deploy to production
vercel --prod
```

6. **Copy Your Backend URL**
   ```
   https://physical-ai-textbook-backend.vercel.app
   ```

---

## Part 2: Update Frontend Config (2 minutes)

### Update Backend URL

Edit: `frontend/docusaurus.config.js`

**Line 30**: Update with your Vercel backend URL

```javascript
customFields: {
  BACKEND_API_URL: process.env.BACKEND_API_URL || 'https://physical-ai-textbook-backend.vercel.app/api/v1',
},
```

**Note**: The config is already set for `umernasir1` on lines 23, 35-36. No changes needed there!

---

## Part 3: Push to GitHub (3 minutes)

### First Time Setup

```bash
# Navigate to project root
cd "D:\PIAIC Batch 76\Hackaton"

# Check current branch
git branch

# Add all files
git add .

# Commit
git commit -m "🚀 Deploy Physical AI Textbook to Vercel + GitHub Pages"

# Add remote (if not already added)
git remote add origin https://github.com/umernasir1/physical-ai-textbook.git

# Push to main branch
git push -u origin setup-backend:main
```

### Subsequent Updates

```bash
git add .
git commit -m "Update: description of changes"
git push
```

---

## Part 4: Enable GitHub Pages (1 minute)

1. **Go to Repository Settings**
   - Visit: https://github.com/umernasir1/physical-ai-textbook
   - Click "Settings" tab
   - Click "Pages" in sidebar

2. **Configure GitHub Pages**
   - **Source**: Deploy from a branch
   - **Branch**: `gh-pages` / `root`
   - Click "Save"

3. **Wait for Deployment**
   - Go to "Actions" tab
   - Watch the "Deploy to GitHub Pages" workflow
   - Takes ~5 minutes first time
   - Once complete (green ✓), your site is live!

---

## Part 5: Update Vercel CORS (2 minutes)

After GitHub Pages is live, update backend CORS:

1. **Go to Vercel Dashboard**
   - Find your backend project
   - Click "Settings" → "Environment Variables"

2. **Redeploy Backend**
   - Go to "Deployments" tab
   - Click "..." on latest deployment
   - Click "Redeploy"

Or push any change to backend to trigger redeploy.

---

## ✅ Verification Checklist

### Backend (Vercel) ✓

Test your backend:
```bash
curl https://physical-ai-textbook-backend.vercel.app/api/v1/
```

Should return:
```json
{"message":"Welcome to the FastAPI Backend!"}
```

Visit API docs:
```
https://physical-ai-textbook-backend.vercel.app/docs
```

### Frontend (GitHub Pages) ✓

1. **Homepage**
   ```
   https://umernasir1.github.io/physical-ai-textbook/
   ```
   Should load the main page

2. **Auth Page**
   ```
   https://umernasir1.github.io/physical-ai-textbook/auth
   ```
   Should show login/signup form

3. **Test Authentication**
   - Sign up with an email
   - Should create account
   - Login should work
   - Redirect to homepage after login

4. **Test Features**
   - Chatbot should respond
   - Translation should work
   - Documentation pages load

---

## 🐛 Troubleshooting

### Backend Issues

**Error: Module not found**
- Check `requirements.txt` includes all dependencies
- Redeploy from Vercel dashboard

**Error: Environment variables not set**
- Go to Vercel → Settings → Environment Variables
- Add missing variables
- Redeploy

**Error: CORS**
- Ensure `https://umernasir1.github.io` is in `origins` list
- Check `backend/src/main.py` line 12
- Redeploy backend

### Frontend Issues

**404 Not Found**
- Check repository name matches `physical-ai-textbook`
- Verify `baseUrl: '/physical-ai-textbook/'` in config
- Wait 5-10 minutes after first deployment

**Auth Page Error**
- Open browser console (F12)
- Check if backend URL is correct
- Verify CORS is configured
- Check Network tab for failed requests

**Features Not Working**
- Verify backend URL in `docusaurus.config.js`
- Check browser console for errors
- Ensure backend is deployed and running

---

## 🎉 Success!

Your app should now be live at:

- **Frontend**: https://umernasir1.github.io/physical-ai-textbook/
- **Backend**: https://physical-ai-textbook-backend.vercel.app

Share your project with the world! 🚀

---

## 📱 Post-Deployment

### Monitor Your App

1. **Vercel Dashboard**
   - View backend logs
   - Monitor API usage
   - Check deployment status

2. **GitHub Actions**
   - View frontend deployment logs
   - See build history

### Update Your App

**Backend Changes**:
```bash
cd backend
# Make changes
git add .
git commit -m "Update backend"
git push
# Vercel auto-deploys!
```

**Frontend Changes**:
```bash
cd frontend
# Make changes
git add .
git commit -m "Update frontend"
git push
# GitHub Actions auto-deploys!
```

---

## 🔒 Security Notes

1. **Never commit `.env` files**
   - Already in `.gitignore`
   - Use Vercel environment variables

2. **Rotate API Keys**
   - Periodically update keys
   - Use Vercel dashboard to update

3. **Monitor Usage**
   - Check Groq API quota
   - Monitor Vercel function invocations
   - Watch Qdrant storage

---

## 💰 Cost

Everything is **FREE** on the free tiers:
- ✅ GitHub Pages: Free forever
- ✅ Vercel: Free tier (100GB bandwidth/month)
- ✅ Groq: Free tier available
- ✅ Qdrant: Free tier (1GB storage)
- ✅ Neon: Free tier (0.5GB storage)

---

## 🎓 Next Steps

1. **Custom Domain** (Optional)
   - Buy domain from Namecheap/Google Domains
   - Configure in GitHub Pages settings
   - Add to Vercel for backend

2. **Analytics**
   - Add Google Analytics
   - Monitor user engagement
   - Track feature usage

3. **Performance**
   - Enable Vercel caching
   - Optimize frontend bundle
   - Add service worker

Happy Deploying! 🚀
