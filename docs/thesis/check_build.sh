#!/bin/sh
# Build thesis_draft.tex in DIR (default: this folder) and check the log.
# Exit codes: 0 = clean build; 1 = LaTeX error or undefined reference/citation in the sources;
#             2 = tooling problem (latexmk missing, or latexmk failed without a LaTeX error in the log).
dir=${1:-$(dirname "$0")}
command -v latexmk >/dev/null 2>&1 || { echo "check_build: latexmk not found"; exit 2; }
cd "$dir" || exit 2
latexmk -pdf -bibtex -interaction=nonstopmode thesis_draft.tex >/dev/null 2>&1
rc=$?
log=thesis_draft.log
[ -f "$log" ] || { echo "check_build: no $log written (latexmk exit $rc)"; exit 2; }
if grep -a -q '^! ' "$log"; then
  echo "check_build: LaTeX error(s) in $dir/$log:"; grep -a -A3 '^! ' "$log" | head -20; exit 1
fi
bad=$(grep -a -E "(Citation|Reference) .* undefined|There were undefined references|Rerun to get|Label\(s\) may have changed" "$log")
if [ -n "$bad" ]; then echo "check_build: unresolved references/citations:"; echo "$bad" | head -20; exit 1; fi
if [ -f thesis_draft.blg ] && grep -a -q 'error message' thesis_draft.blg; then
  echo "check_build: BibTeX error(s):"; grep -a -B2 'error message' thesis_draft.blg | head -20; exit 1
fi
[ "$rc" -eq 0 ] || { echo "check_build: latexmk exit $rc without a LaTeX error in the log"; exit 2; }
echo "check_build: OK ($(grep -a -o 'Output written.*' "$log"))"
exit 0
