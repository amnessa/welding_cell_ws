#!/usr/bin/env bash
# Build the thesis into build/thesis.pdf: pdflatex, bibtex, pdflatex x2.
# Needs (Ubuntu 24.04): texlive-latex-base texlive-latex-recommended texlive-latex-extra
#   texlive-science texlive-fonts-recommended texlive-lang-greek texlive-lang-european
#   texlive-plain-generic  (metu.cls loads the Greek LGR encoding and Turkish babel)
set -e
cd "$(dirname "$0")"
mkdir -p build
run() { pdflatex -interaction=nonstopmode -halt-on-error -output-directory=build thesis.tex > build/pdflatex.out 2>&1 \
          || { grep -a -A5 "^! " build/thesis.log; exit 1; }; }
run
(cd build && BIBINPUTS=..: bibtex -terse thesis)
run
run
grep -a "Output written" build/pdflatex.out
grep -a -E "LaTeX Warning: (Reference|Citation).*undefined" build/thesis.log | sed 's/ on input line.*//' | sort -u || true
