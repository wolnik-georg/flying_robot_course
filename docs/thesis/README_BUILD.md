# Building the thesis draft

- Manual build: `make -C docs/thesis` (PDF), `make -C docs/thesis check` (fails on LaTeX errors or
  undefined references/citations), `make -C docs/thesis clean`. Configuration: `docs/thesis/.latexmkrc`.
- Automatic rebuild on commit, enable once per clone (hooks are per clone, not pushed as active):
  `git config core.hooksPath .githooks`
- The hook `.githooks/pre-commit` runs only when a commit stages `docs/thesis/*.tex` or
  `docs/references.bib`: it builds the staged sources in a temporary folder and adds
  `docs/thesis/thesis_draft.pdf` to the commit. Missing LaTeX: warning, commit continues.
  LaTeX errors or undefined references/citations: commit blocked (`git commit --no-verify` skips).
