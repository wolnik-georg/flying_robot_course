# latexmk configuration for the thesis draft: `latexmk` in this folder is the whole build.
$pdf_mode = 1;                 # pdflatex
$bibtex_use = 2;               # run bibtex when needed; `latexmk -C` also removes the .bbl
$pdflatex = 'pdflatex -interaction=nonstopmode %O %S';
@default_files = ('thesis_draft.tex');
