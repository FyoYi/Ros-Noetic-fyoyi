---
name: chinese-commit-message
description: Use this skill when the user asks to generate, write, suggest, or format a git commit message for the current changes, staged changes, or this change set in this project.
---

# Chinese Commit Message

When generating a git commit message:

- Always write the commit message in Simplified Chinese.
- Output exactly one final commit message and nothing else.
- Do not output English.
- Do not output explanations.
- Do not output bullet points.
- Keep the subject concise, natural, and professional in Chinese.
- Pick the single most appropriate type based on the main intent of the staged changes.

Use exactly this format:

```text
<emoji> <type>: <中文简短描述>
```

Allowed type and emoji pairs:

```text
✨ feat
🐛 fix
📝 docs
♻️ refactor
⚡ perf
🧑‍💻 dx
🔨 workflow
🏷️ types
🚧 wip
✅ test
🔨 build
👷 ci
❓ chore
⬆️ deps
🔖 release
```
