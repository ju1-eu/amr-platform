# =============================================================================
# Makefile (latexmk, XeLaTeX, Build-Ordner) – thesis-grade
# =============================================================================

PROJECT   ?= main
SRC       := $(PROJECT).tex
OUTDIR    ?= build

LATEXMK   ?= latexmk
ENGINE    ?= -xelatex

LATEXMKFLAGS := $(ENGINE) -synctex=1 -interaction=nonstopmode -halt-on-error -file-line-error -outdir=$(OUTDIR)

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
  VIEWER ?= open
else
  VIEWER ?= xdg-open
endif

.PHONY: all pdf view watch clean distclean check

all: pdf

pdf: $(SRC)
	@mkdir -p $(OUTDIR)
	$(LATEXMK) $(LATEXMKFLAGS) $(SRC)

view: pdf
	$(VIEWER) $(OUTDIR)/$(PROJECT).pdf &

watch: $(SRC)
	@mkdir -p $(OUTDIR)
	$(LATEXMK) $(LATEXMKFLAGS) -pvc -view=none $(SRC)

clean:
	$(LATEXMK) -c -outdir=$(OUTDIR) $(SRC)

distclean:
	$(LATEXMK) -C -outdir=$(OUTDIR) $(SRC)
	@rm -rf $(OUTDIR)

check:
	@echo "Prüfe Voraussetzungen..."
	@command -v $(LATEXMK) >/dev/null 2>&1 && echo "  ✓ latexmk gefunden" || echo "  ✗ latexmk FEHLT"
	@command -v xelatex >/dev/null 2>&1 && echo "  ✓ xelatex gefunden" || echo "  ✗ xelatex FEHLT"
	@test -f $(SRC) && echo "  ✓ $(SRC) gefunden" || echo "  ✗ $(SRC) NICHT gefunden"
