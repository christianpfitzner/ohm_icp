# Build the handout: make (latexmk -pdf). TikZ figures in figures/, no external graphics.
LATEXMK := latexmk

icp.pdf: icp.tex $(wildcard figures/*.tikz figures/*.txt)
	$(LATEXMK) -pdf -interaction=nonstopmode icp.tex

.PHONY: log clean

log:
	@grep -nE "^!|Overfull \\\\hbox \(([1-9][0-9]|[0-9]+[0-9])pt|Undefined control|LaTeX Warning: (Reference|Citation)" icp.log || echo "log clean"

clean:
	-$(LATEXMK) -c icp.tex >/dev/null 2>&1
