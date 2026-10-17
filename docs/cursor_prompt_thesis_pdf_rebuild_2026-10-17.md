# Cursor prompt — rebuild `docs/thesis/thesis_draft.pdf` and make the rebuild automatic (2026-10-17)

You are working in the repository `~/Desktop/flying_robot_course` (branch `main`). Do the two tasks below. Report what you did; do not change any thesis text.

## Context (verified by me, do not re-derive)
- The combined manuscript is built from `docs/thesis/thesis_draft.tex`, which `\input`s `preamble_draft.tex` and `ch1_introduction.tex` … `ch9_conclusion.tex` (all in `docs/thesis/`) and uses `\bibliography{../references}` (`docs/references.bib`, classic BibTeX, style `plain`; no biber).
- The tracked PDF `docs/thesis/thesis_draft.pdf` is **stale**: built 2026-09-27 16:29, 50 pages. The sources changed after that (commit `9769745a` 2026-10-04 draft-quality fixes, commit `670497ee` 2026-10-15 "merge rounds 1-3 + closing edits"). Every `.tex` chapter is newer than the PDF.
- A scratch build of the current sources works: `latexmk -pdf -interaction=nonstopmode -outdir=<dir> thesis_draft.tex` → exit 0, **64 pages**, no LaTeX errors, no undefined references or citations, 11 "Overfull" warnings. Installed: `latexmk`, `pdflatex`, `bibtex` (`/usr/bin`).
- `.gitignore` ignores everything under `docs/thesis/` except `*.tex`, `CITATION_BASELINE.md`, `CITATION_EDIT_LOG.md`, `reserve/` and **`thesis_draft.pdf`**, which is tracked on purpose "so each citation-audit tag pins the exact PDF that the tagged sources produced". Five stale `chN_standalone.pdf` files are also tracked although the ignore rules say they should not be (leave them tracked and untouched; do not rebuild or delete them unless I ask).
- There is no Makefile, no `.latexmkrc`, no git hook, no CI for the thesis.

## Task A — rebuild the PDF now
1. Build in the repo with latexmk (default output dir `docs/thesis/`), clean build first (`latexmk -C` for `thesis_draft` only, then a full build) so no stale `.aux/.bbl` is reused.
2. Check the log: no `! ` errors, no `Citation ... undefined`, no `Reference ... undefined`, no `Rerun to get cross-references right` left over. Report the page count (expected 64) and the number of Overfull/Underfull boxes; list the Overfull ones with the line numbers (do not fix them).
3. Open the first pages as text (`pdftotext -f 1 -l 3`) and check that the table of contents lists chapters 1–9.
4. Commit only `docs/thesis/thesis_draft.pdf` (message: `Thesis draft PDF rebuilt from the sources of <short hash>`). Do not commit aux/log/bbl files.

## Task B — make the rebuild automatic (keep it small)
1. `docs/thesis/.latexmkrc` (or equivalent): `$pdf_mode = 1; $bibtex_use = 2;` nonstop mode, so `latexmk thesis_draft.tex` in that folder is the whole build.
2. `docs/thesis/Makefile` with targets `pdf` (default), `clean`, `check` (fails on LaTeX errors or undefined refs/citations by grepping `thesis_draft.log`).
3. A tracked git hook directory `.githooks/` with a `pre-commit` script that, **only when staged files match `docs/thesis/*.tex` or `docs/references.bib`**, runs `make -C docs/thesis check`-style build, and on success `git add docs/thesis/thesis_draft.pdf` so the PDF in a commit always matches the sources in that commit. Rules for the hook: never touch anything else; if LaTeX is missing or the build fails, print a clear message and **do not block the commit** (exit 0) unless the failure is a real LaTeX error in the sources, in which case print the error and block with a clear way to skip (`git commit --no-verify`). It must run in a few seconds.
4. Document how to enable it in one line in `docs/thesis/README_BUILD.md` (new, 15 lines max): `git config core.hooksPath .githooks`, plus the manual command `make -C docs/thesis`. Run that `git config` line yourself in this clone. Mention that hooks are per clone.
5. Do NOT add CI/GitHub Actions, do NOT change `.gitignore`, do NOT touch the citation-audit tags, `CITATION_BASELINE.md`, `CITATION_EDIT_LOG.md`, or any `.tex` content.

## Acceptance (tell me each as pass/fail with the evidence)
- `docs/thesis/thesis_draft.pdf` is newer than every `.tex` in `docs/thesis/`, 64 pages (or state why different), TOC has chapters 1–9, zero undefined references/citations.
- Test the hook: (a) stage a whitespace-only change in one `.tex`, commit → the commit contains the rebuilt PDF; (b) commit a file outside `docs/thesis/` → the hook does nothing; (c) simulate a missing `latexmk` (PATH trick) → commit is not blocked. Then undo the test edits (`git reset`/revert) so only the intended commits remain.
- `git status` clean afterwards except files that were untracked before you started (do not touch them).

## Report format
A short list: what was built (hash of sources, pages, warnings), the files you added, the three hook tests with results, anything surprising. Do not push; I will review and push.
