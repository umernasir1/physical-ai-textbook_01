---
id: 5
title: Set Output Format for Tasks
stage: general
date: 2025-12-01
surface: agent
model: gemini-pro
feature: none
branch: master
user: User
command: Structure the output in a readable, organized format, preferably as a table or list, so the development team can immediately start working and track progress. Follow all rules, constraints, workflow, and roles defined in the Constitution and Implementation Plan
labels: ["agent-config", "quality-guidelines", "output-formatting"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  -
tests:
  -
---

## Prompt

Structure the output in a readable, organized format, preferably as a table or list, so the development team can immediately start working and track progress. Follow all rules, constraints, workflow, and roles defined in the Constitution and Implementation Plan

## Response snapshot

I will ensure all task-related outputs are structured in a clear, readable format like a table or list to facilitate immediate action and progress tracking by the development team, while adhering to all established rules in the Constitution and Implementation Plan.

## Outcome

- ✅ Impact: Established a clear requirement for the final output format of task generation, emphasizing readability and immediate utility for the development team.
- 🧪 Tests: No tests were run as this was a configuration/instructional prompt.
- 📁 Files: No files were modified.
- 🔁 Next prompts: Ready to generate tasks that conform to this output structure.
- 🧠 Reflection: This completes a series of meta-instructions defining the entire workflow from specification to planning to task generation. The emphasis is consistently on structure, clarity, and adherence to foundational documents.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): n/a
