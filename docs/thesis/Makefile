# Thesis draft build. `make` (= make pdf) builds thesis_draft.pdf here; `make check` builds and
# fails on LaTeX errors or undefined references/citations; `make clean` removes build files.
.PHONY: pdf clean check
pdf:
	latexmk thesis_draft.tex
clean:
	latexmk -C thesis_draft.tex
check:
	./check_build.sh .
