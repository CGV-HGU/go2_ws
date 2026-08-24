---
name: go2-antarctica-workspace
description: Load reviewed system context and safety constraints for ESCAPE-Nav work in the Unitree Go2 Antarctica workspace. Use for architecture, diagnostics, deployment planning, or real-robot experiments in this repository.
---

# Go2 Antarctica workspace

Use the repository's reviewed Memory instead of preserving a second copy of
fast-changing topology or readiness claims in this skill.

## Required context

1. Read `/home/unitree/go2_ws_antarctica/AGENTS.md` for durable evidence,
   repository, and physical-safety rules.
2. Read `/home/unitree/go2_ws_antarctica/docs/CODEX_PROJECT_MEMORY.md` for the
   audited architecture, live-state timestamp, canonical branch relationship,
   implementation status, blockers, and safe preflight commands.
3. Revalidate volatile robot, ROS, Docker, network, VLM, and Git state before
   making a current-health claim.

## Boundaries

- Treat the system as an integration prototype and keep physical motion
  disabled unless the user explicitly authorizes a supervised physical test.
- Do not run legacy one-click or micro-motion scripts as diagnostics.
- Do not infer readiness from mock, synthetic, loopback-only, dashboard, or
  process-list evidence.
- Never store or reveal credentials in skill, Memory, logs, or chat output.
- Use canonical `main` as the software baseline. Treat `paper` as a divergent
  manuscript/experiment branch and inspect both exact commits when comparison
  matters.
