---
description: Manage translation system - test translations, validate Urdu output, optimize translation API, and monitor quota
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

### Purpose
This subagent manages the Urdu translation feature, ensuring accurate translations and optimal API usage.

### Capabilities
1. **Test Translation**: Validate translation quality for chapters
2. **Batch Translate**: Pre-translate all chapters to Urdu
3. **Optimize API**: Monitor Groq usage and optimize token consumption
4. **Validate Output**: Check Urdu text rendering and formatting
5. **Debug Issues**: Fix translation errors and API problems

### Execution Steps

1. **Understand Request**:
   - Parse action: test | batch | optimize | validate | debug
   - If no action, run translation health check

2. **Test Translation**:
   - Select test chapters (1 from each module)
   - For each chapter:
     - Extract first 500 characters
     - Send to `/api/v1/translation/translate`
     - Payload: `{"text": "...", "target_language": "ur"}`
     - Verify response contains Urdu text
     - Check for proper Urdu script (Unicode range: U+0600 to U+06FF)
     - Validate no mock text returned
   - Report: Translation success rate, quality assessment

3. **Batch Translate** (if requested):
   - List all chapters in `frontend/docs/module*/`
   - For each chapter:
     - Read full content
     - Split into chunks (max 4000 tokens per request)
     - Translate each chunk
     - Reassemble translated content
     - Save to `frontend/docs/module*/ur/<filename>`
   - Track progress, handle rate limits
   - Report: Chapters translated, tokens used, time taken

4. **Optimize API**:
   - Monitor Groq API usage:
     - Total requests today
     - Token consumption
     - Rate limit status
   - Test optimal chunk sizes:
     - 500, 1000, 2000, 4000 tokens
     - Measure quality vs speed tradeoff
   - Recommend settings:
     - Best chunk size
     - Ideal batch size
     - Rate limiting strategy
   - Report: Current usage, optimization suggestions

5. **Validate Output**:
   - Check translated Urdu text:
     - Proper Unicode encoding (UTF-8)
     - Correct directionality (RTL)
     - No garbled characters
     - Maintains markdown formatting
     - Code blocks preserved
   - Test in frontend:
     - Verify UI displays Urdu correctly
     - Check font rendering
     - Validate text selection works
   - Report: Validation results, rendering issues

6. **Debug Issues**:
   - Common problems:
     - Backend not running
     - Mock text showing (backend unreachable)
     - API key invalid
     - Rate limit exceeded
     - Poor translation quality
     - Encoding issues
   - For each issue:
     - Diagnose root cause
     - Check backend logs
     - Verify API connection
     - Test direct API call
     - Apply fix if possible
   - Report: Issues found, fixes applied, manual steps

7. **Report Summary**:
   - Translation system status: Operational | Issues | Offline
   - API health: Groq connected, rate limits OK
   - Chapters ready for translation: X/Y
   - Translation quality: Good | Fair | Poor
   - Issues: N found, M fixed
   - Next actions if needed

### Usage Examples

```bash
# Test translation
/hackathon.translate test

# Batch translate all chapters
/hackathon.translate batch

# Optimize API usage
/hackathon.translate optimize

# Validate Urdu output
/hackathon.translate validate

# Debug translation issues
/hackathon.translate debug
```

### Success Criteria
- Translation API responds successfully
- Urdu text renders correctly in UI
- No mock text shown
- Translation quality is good
- API usage within quota
- No encoding errors
