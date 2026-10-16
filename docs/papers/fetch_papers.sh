#!/bin/bash
# Rebuild the local paper library from docs/references.bib.
#
# The PDFs are not tracked in git: 48 MB of third-party papers is not ours to redistribute, and
# every entry in references.bib carries its arXiv id, so the library is reproducible. Run this
# after a fresh clone.
#
#   ./fetch_papers.sh
#
# Requires: curl, python3. Skips anything already downloaded.

cd "$(dirname "$0")" || exit 1
BIB=../references.bib
[ -f "$BIB" ] || { echo "no $BIB"; exit 1; }

python3 - "$BIB" <<'PY' | while IFS='|' read -r key id; do
import re,sys
s=open(sys.argv[1]).read()
for m in re.finditer(r'@article\{([^,]+),(.*?)\n\}', s, re.S):
    e=re.search(r'eprint\s*=\s*\{([^}]+)\}', m.group(2))
    if e: print(f"{m.group(1)}|{e.group(1)}")
PY
  if [ -f "$key.pdf" ]; then echo "  have $key.pdf"; continue; fi
  curl -sSL -o "$key.pdf" "https://arxiv.org/pdf/$id"
  sz=$(stat -c%s "$key.pdf" 2>/dev/null || echo 0)
  if [ "$sz" -gt 40000 ] && head -c4 "$key.pdf" | grep -q "%PDF"; then
    printf "  OK   %-34s %5s KB\n" "$key.pdf" "$((sz/1024))"
  else
    printf "  FAIL %-34s\n" "$key.pdf"; rm -f "$key.pdf"
  fi
  sleep 3
done
