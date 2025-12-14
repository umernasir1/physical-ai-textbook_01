# ✅ Deployment Checklist - umernasir1

Follow this checklist step by step for successful deployment!

---

## 📋 Pre-Deployment (5 minutes)

### API Keys Ready?
- [ ] GROQ_API_KEY (from https://console.groq.com/keys)
- [ ] QDRANT_URL (from https://cloud.qdrant.io/)
- [ ] QDRANT_API_KEY (from Qdrant dashboard)
- [ ] NEON_DATABASE_URL (from https://neon.tech/)
- [ ] SECRET_KEY (generate: `openssl rand -hex 32`)

### Accounts Created?
- [ ] Vercel account (https://vercel.com - sign up with GitHub)
- [ ] GitHub repository exists: `umernasir1/physical-ai-textbook`

---

## 🚀 Step 1: Deploy Backend to Vercel (5 min)

### Using Vercel Dashboard

- [ ] Go to https://vercel.com
- [ ] Sign in with GitHub
- [ ] Click "Add New..." → "Project"
- [ ] Import: `umernasir1/physical-ai-textbook`
- [ ] Set Root Directory: `backend`
- [ ] Add all 6 environment variables
- [ ] Click "Deploy"
- [ ] Wait for deployment to complete
- [ ] Copy backend URL: `https://_____.vercel.app`

**Backend URL**: _________________________________

---

## 📝 Step 2: Update Frontend Config (2 min)

- [ ] Open: `frontend/docusaurus.config.js`
- [ ] Line 30: Update `BACKEND_API_URL` with your Vercel URL
- [ ] Save file

```javascript
BACKEND_API_URL: 'https://YOUR_VERCEL_URL.vercel.app/api/v1',
```

---

## 💾 Step 3: Commit and Push to GitHub (3 min)

Run these commands in your terminal:

```bash
cd "D:\PIAIC Batch 76\Hackaton"
```

- [ ] Run: `git add .`
- [ ] Run: `git commit -m "🚀 Deploy to Vercel + GitHub Pages"`
- [ ] Run: `git remote add origin https://github.com/umernasir1/physical-ai-textbook.git`
- [ ] Run: `git push -u origin setup-backend:main`

**Note**: If you get an error about remote already existing, skip the `git remote add` command.

---

## 🌐 Step 4: Enable GitHub Pages (1 min)

- [ ] Go to: https://github.com/umernasir1/physical-ai-textbook
- [ ] Click "Settings" tab
- [ ] Click "Pages" in left sidebar
- [ ] Source: "Deploy from a branch"
- [ ] Branch: Select `gh-pages` / `root`
- [ ] Click "Save"

---

## ⏰ Step 5: Wait for Deployment (5 min)

- [ ] Go to: https://github.com/umernasir1/physical-ai-textbook/actions
- [ ] Watch "Deploy to GitHub Pages" workflow
- [ ] Wait for green checkmark ✓
- [ ] Frontend is now live!

---

## ✅ Step 6: Verify Everything Works

### Test Backend
- [ ] Visit: `https://YOUR_VERCEL_URL.vercel.app/api/v1/`
- [ ] Should show: `{"message":"Welcome to the FastAPI Backend!"}`
- [ ] Visit: `https://YOUR_VERCEL_URL.vercel.app/docs`
- [ ] API docs should load

### Test Frontend
- [ ] Visit: https://umernasir1.github.io/physical-ai-textbook/
- [ ] Homepage loads correctly
- [ ] Visit: https://umernasir1.github.io/physical-ai-textbook/auth
- [ ] Auth page loads without errors

### Test Authentication
- [ ] Sign up with test email
- [ ] Account created successfully
- [ ] Can login with credentials
- [ ] Redirects to homepage after login
- [ ] Token stored in localStorage

### Test Features
- [ ] Chatbot widget appears
- [ ] Chatbot responds to questions
- [ ] Translate button works
- [ ] Documentation pages load
- [ ] No CORS errors in console (F12)

---

## 🎉 Deployment Complete!

**Your Live URLs:**

- **Frontend**: https://umernasir1.github.io/physical-ai-textbook/
- **Backend**: https://_____.vercel.app

---

## 🐛 If Something Doesn't Work

### Backend Issues
1. Check Vercel logs in dashboard
2. Verify all environment variables are set
3. Redeploy from Vercel dashboard

### Frontend Issues
1. Wait 5-10 minutes after first deployment
2. Check GitHub Actions for errors
3. Verify `baseUrl` in docusaurus.config.js
4. Clear browser cache and try again

### CORS Errors
1. Ensure `https://umernasir1.github.io` is in backend CORS origins
2. Check `backend/src/main.py` line 12
3. Redeploy backend after CORS changes

---

## 📱 Share Your Project!

Once everything is working:

- [ ] Share on LinkedIn
- [ ] Share on Twitter/X
- [ ] Add to your portfolio
- [ ] Submit to hackathon

**Share URL**: https://umernasir1.github.io/physical-ai-textbook/

---

## 🔄 Future Updates

**To update backend:**
```bash
# Make changes in backend/
git add .
git commit -m "Update backend"
git push
# Vercel auto-deploys!
```

**To update frontend:**
```bash
# Make changes in frontend/
git add .
git commit -m "Update frontend"
git push
# GitHub Pages auto-deploys!
```

---

**Estimated Total Time**: 15-20 minutes

Good luck! 🚀
