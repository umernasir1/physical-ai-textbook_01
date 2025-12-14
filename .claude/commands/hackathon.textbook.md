---
description: Manage and validate textbook content - check completeness, add chapters, fix formatting, and ensure quality
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

### Purpose
This subagent manages the Physical AI textbook content, ensuring all chapters meet quality standards and hackathon requirements.

### Capabilities
1. **Content Audit**: Check all chapters for completeness and quality
2. **Chapter Validation**: Ensure proper MDX syntax, code formatting, and structure
3. **Add Content**: Generate new chapters or sections based on requirements
4. **Fix Issues**: Resolve MDX errors, broken links, and formatting problems
5. **Quality Check**: Verify code examples, configurations, and technical accuracy

### Execution Steps

1. **Understand Request**:
   - Parse user arguments to determine action: audit | validate | add | fix | check-quality
   - If no action specified, default to comprehensive audit

2. **Content Audit**:
   - List all markdown files in `frontend/docs/module*/`
   - Count lines in each chapter file
   - Identify incomplete chapters (< 100 lines are likely stubs)
   - Check for proper frontmatter in each file
   - Verify all 4 modules are present with expected chapters
   - Report statistics: total chapters, avg lines, completion percentage

3. **Chapter Validation**:
   - For each chapter, check:
     - Valid MDX syntax (no `<` characters before numbers in tables)
     - Proper markdown headings hierarchy
     - Code blocks have language specifiers
     - Internal links are valid
     - No malformed JSX/React components
   - List any errors found with file:line references

4. **Add Content** (if requested):
   - Prompt user for: module, chapter name, topic focus
   - Read existing chapters in same module for style consistency
   - Generate comprehensive chapter (500+ lines) with:
     - Introduction and learning objectives
     - Conceptual explanation with diagrams
     - Code examples (Python, C++, YAML, XML as relevant)
     - Configuration files
     - Launch files for ROS 2
     - Practical exercises
     - Troubleshooting section
     - Summary and next steps
   - Save to appropriate module directory

5. **Fix Issues** (if errors found):
   - Auto-fix common MDX issues:
     - Replace `<` with `&lt;` in tables
     - Replace `>` with `&gt;` in tables where needed
     - Fix unclosed code blocks
     - Correct heading hierarchy
   - Report all fixes made

6. **Quality Check**:
   - Verify technical accuracy:
     - ROS 2 code follows best practices
     - Gazebo/Unity configurations are valid
     - Python code is syntactically correct
     - YAML/XML is well-formed
   - Check for:
     - Consistent terminology
     - Proper code comments
     - Clear explanations
     - Appropriate difficulty progression
   - Score each chapter: A (excellent), B (good), C (needs improvement)

7. **Report Summary**:
   - Total chapters: X/Y complete
   - Issues found: N
   - Issues fixed: M
   - Quality scores by module
   - Recommendations for improvement
   - Next actions if any

### Usage Examples

```bash
# Comprehensive audit
/hackathon.textbook

# Validate specific module
/hackathon.textbook validate module3

# Add new chapter
/hackathon.textbook add module4 "Multi-Robot Coordination"

# Fix MDX errors
/hackathon.textbook fix

# Quality check all content
/hackathon.textbook check-quality
```

### Success Criteria
- All 4 modules have complete chapters (100+ lines each)
- No MDX compilation errors
- Code examples are syntactically valid
- Proper markdown structure
- Technical accuracy maintained
- Consistent style across chapters
