---
description: Run comprehensive testing - test all features, APIs, frontend, backend, and generate test report
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

### Purpose
This subagent runs comprehensive testing of all hackathon features to ensure everything works correctly.

### Capabilities
1. **Feature Testing**: Test all core and bonus features
2. **API Testing**: Validate all backend endpoints
3. **Frontend Testing**: Check UI components and user flows
4. **Integration Testing**: Test end-to-end workflows
5. **Generate Report**: Create comprehensive test report for submission

### Execution Steps

1. **Understand Request**:
   - Parse test scope: all | core | bonus | api | frontend | integration
   - If no scope, run complete test suite

2. **Environment Check**:
   - Verify services running:
     - Backend: `http://localhost:8000` (or ENV variable)
     - Frontend: `http://localhost:3000`
     - Qdrant: connection test
     - Neon DB: connection test
   - Check API keys configured:
     - GROQ_API_KEY
     - QDRANT_API_KEY
     - NEON_DATABASE_URL
   - Report: Environment status, missing services

3. **Test Core Features (100 points)**:

   **Feature 1: Comprehensive Textbook (40 points)**
   - [ ] All 4 modules present
   - [ ] Module 1: 4 chapters minimum
   - [ ] Module 2: 3 chapters minimum
   - [ ] Module 3: 4 chapters minimum
   - [ ] Module 4: 4 chapters minimum
   - [ ] All chapters > 100 lines (not stubs)
   - [ ] Proper markdown syntax
   - [ ] Code examples present
   - [ ] No MDX errors
   - **Score**: X/40

   **Feature 2: RAG Chatbot (40 points)**
   - [ ] Backend API responds
   - [ ] POST `/api/v1/chat` works
   - [ ] Vector search returns results
   - [ ] Groq AI generates responses
   - [ ] Context is relevant
   - [ ] Chatbot widget appears in frontend
   - [ ] User can ask questions
   - [ ] Responses are coherent
   - **Score**: X/40

   **Feature 3: GitHub Pages Deployment (20 points)**
   - [ ] `.github/workflows/deploy.yml` exists
   - [ ] Workflow configured correctly
   - [ ] `docusaurus.config.js` has correct URL
   - [ ] Build succeeds locally
   - [ ] Deployment can trigger
   - **Score**: X/20

   **Core Features Total**: X/100

4. **Test Bonus Features (up to 200 points)**:

   **Feature 1: Authentication (50 points)**
   - [ ] POST `/api/v1/auth/signup` works
   - [ ] POST `/api/v1/auth/token` (login) works
   - [ ] User data saved to Neon DB
   - [ ] JWT token returned
   - [ ] Auth UI component renders
   - [ ] Signup flow works
   - [ ] Login flow works
   - [ ] Background questionnaire saves
   - **Score**: X/50

   **Feature 2: Content Personalization (50 points)**
   - [ ] User profile stored with background
   - [ ] Personalization service exists
   - [ ] User software/hardware level captured
   - [ ] Responses can be tailored (implementation)
   - **Score**: X/50

   **Feature 3: Urdu Translation (50 points)**
   - [ ] POST `/api/v1/translation/translate` works
   - [ ] Urdu translation returns (not mock)
   - [ ] Translation UI component renders
   - [ ] "Translate to Urdu" button works
   - [ ] Urdu text displays correctly (UTF-8)
   - [ ] Translation quality is good
   - **Score**: X/50

   **Feature 4: Claude Code Subagents (50 points)**
   - [ ] `.claude/commands/` directory exists
   - [ ] Custom slash commands defined
   - [ ] `/hackathon.textbook` command works
   - [ ] `/hackathon.rag` command works
   - [ ] `/hackathon.translate` command works
   - [ ] `/hackathon.deploy` command works
   - [ ] `/hackathon.test` command works
   - [ ] Subagents are reusable
   - [ ] Documentation exists
   - **Score**: X/50

   **Bonus Features Total**: X/200

5. **API Endpoint Testing**:
   - Test each endpoint with curl/requests:
     ```bash
     # Chat
     POST /api/v1/chat
     {"query": "What is ROS 2?", "context": ""}
     Expected: 200, {"response": "...", "sources": [...]}

     # Auth Signup
     POST /api/v1/auth/signup
     {"email": "test@example.com", "password": "Test123!", ...}
     Expected: 201, {"user": {...}, "token": "..."}

     # Auth Login
     POST /api/v1/auth/token
     {"username": "test@example.com", "password": "Test123!"}
     Expected: 200, {"access_token": "...", "token_type": "bearer"}

     # Translation
     POST /api/v1/translation/translate
     {"text": "Hello World", "target_language": "ur"}
     Expected: 200, {"translated_content": "..."}
     ```
   - Report: Endpoint status, response times, errors

6. **Frontend Component Testing**:
   - Navigate to `http://localhost:3000`
   - Test components:
     - [ ] Chatbot widget loads
     - [ ] Chatbot accepts input
     - [ ] Chatbot displays responses
     - [ ] Auth modal opens
     - [ ] Signup form works
     - [ ] Login form works
     - [ ] Translate button appears
     - [ ] Translation modal works
     - [ ] All modules accessible
     - [ ] All chapters load
   - Check console for errors
   - Report: Component status, UI issues

7. **Integration Testing**:
   - Test complete user flows:
     **Flow 1: New User Journey**
     1. Open site
     2. Click Sign Up
     3. Create account with background info
     4. Navigate to Module 1
     5. Ask chatbot a question
     6. Get relevant response
     7. Translate chapter to Urdu
     8. View translated content

     **Flow 2: Content Discovery**
     1. Browse all 4 modules
     2. Select random chapters
     3. Ask questions about content
     4. Verify chatbot uses correct context
     5. Test text selection query

   - Report: Flow success/failure, bottlenecks, errors

8. **Generate Test Report**:
   - Create comprehensive report:
     ```markdown
     # Hackathon Test Report

     ## Summary
     - Test Date: YYYY-MM-DD
     - Total Score: X/300 points
     - Pass Rate: X%

     ## Core Features (100 points)
     - Textbook: X/40 ✅
     - RAG Chatbot: X/40 ✅
     - Deployment: X/20 ✅

     ## Bonus Features (200 points)
     - Authentication: X/50 ✅
     - Personalization: X/50 ✅
     - Translation: X/50 ✅
     - Subagents: X/50 ✅

     ## Detailed Results
     [Feature-by-feature breakdown]

     ## Issues Found
     - Issue 1: Description, Severity, Status
     - Issue 2: ...

     ## Recommendations
     - Recommendation 1
     - Recommendation 2

     ## Conclusion
     Project is [READY | NEEDS WORK] for submission
     ```
   - Save to `HACKATHON_TEST_REPORT.md`

9. **Report Summary**:
   - Total Score: X/300 points
   - Pass Rate: X%
   - Critical Issues: N
   - Features Working: X/9
   - Deployment Ready: Yes/No
   - Recommended Actions

### Usage Examples

```bash
# Run complete test suite
/hackathon.test

# Test only core features
/hackathon.test core

# Test only bonus features
/hackathon.test bonus

# Test specific feature
/hackathon.test feature translation

# Generate report only
/hackathon.test report
```

### Success Criteria
- All core features pass (100/100)
- Most bonus features pass (≥150/200)
- No critical bugs
- All APIs respond correctly
- Frontend loads without errors
- Integration flows complete
- Test report generated
