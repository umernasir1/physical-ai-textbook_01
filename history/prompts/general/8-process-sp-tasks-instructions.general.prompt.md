---
id: 8
title: Process /sp.tasks Instructions
stage: general
date: 2025-12-02
surface: agent
model: gemini-pro
feature: none
branch: 1-system-specification
user: User
command: |
  ---
  description: Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts.
  ---
  ... (full /sp.tasks instructions) ...
labels: ["agent-config", "task-generation"]
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

... (full prompt text) ...

## Response snapshot

I have received and processed the instructions for the `/sp.tasks` command. I will follow the outlined workflow to generate a detailed, actionable `tasks.md` file from the existing design artifacts, organized by user story and phase.

## Outcome

- ✅ Impact: Defined the workflow for breaking down a technical plan into a granular, executable list of tasks.
- 🧪 Tests: No tests were run as this was a configuration/instructional prompt.
- 📁 Files: No files were modified.
- 🔁 Next prompts: Ready to generate tasks for the current feature.
- 🧠 Reflection: This is the final piece of the primary workflow instructions. The system now has a complete set of rules for going from a high-level feature request to a detailed implementation plan with discrete tasks.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): n/a
