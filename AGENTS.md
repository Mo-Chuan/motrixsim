# AGENTS.md

Guidance for AI agents working in this repository.

## What this repo is

Sphinx documentation for MotrixSim (physics simulation engine), plus runnable examples under `examples/`. Bilingual: Chinese (`docs/source/zh_CN/`) and English (`docs/source/en/`).

## README writing skill

When writing, reviewing, or restructuring a README, first read
[technical-readme](.agents/skills/technical-readme/SKILL.md), then load the relevant
references it names. Use technical-report-style language, a vertical table of
contents, and evidence-led feature narratives. Determine project positioning,
feature order, sources, and release requirements from the current task and
repository materials; keep the skill itself project-independent.

## Python environment

- This project is managed with **uv**. Always use `uv run <cmd>` or `.venv/bin/python`.
- **Never** use system/conda Python — the repo `.venv` pins the correct SDK version (Python 3.10, motrixsim 0.10.x).
- Setup: `uv sync`.

## Common commands

```bash
# run an example
uv run python examples/control/car.py

# test all examples (long)
uv run python scripts/test_examples.py

# build docs (run from docs/)
cd docs && uv run make html

# check for newly broken examples (subset), e.g.
uv run python examples/bench/equality/accuracy.py --seconds 1
```

## Docs editing rules

- Edit only `docs/source/zh_CN/` and `docs/source/en/`. `docs/source/user_guide/` is auto-generated at build time — never edit it directly.
- Keep `zh_CN/` and `en/` content in sync when changing docs.
- Details (build config, RST/docstring conventions): see `docs/AGENTS.md`.

## Commit hygiene

- Never commit scratch or process documents: asset provenance notes, debug logs, draft checklists, personal TODOs, generated artifacts. These belong outside the repo (or under a path ignored by `.gitignore`, e.g. `*.local.md`).
- Before committing, run `git status` and diff the staged set: every staged file should be something a reviewer of this MR needs.

## Gotchas

- Examples may use `pathlib.Path` for asset paths; the repo's `.venv` SDK supports it. If you see `PosixPath cannot be converted to 'PyString'`, you are running the wrong Python, not hitting an API bug.
- `examples/assets/equality/` mirrors copies in the motrixsim-python repo's Rust integration tests — keep both in sync when editing.
