OPENSCAD_CHECK := $(shell command -v openscad 2> /dev/null)
PDFLATEX_CHECK := $(shell command -v pdflatex 2> /dev/null)

all: check-deps build/tag_plate.stl build/april_tag_print.pdf

check-deps:
ifndef OPENSCAD_CHECK
	$(error "OpenSCAD is required. Please install: sudo apt install openscad")
endif
ifndef PDFLATEX_CHECK
	$(error "pdflatex is required. Please install: sudo apt install texlive-base")
endif

build/tag_plate.stl: tag_plate.scad
	mkdir -p build
	openscad -o $@ $<
	echo "Generated $@"

build/tag_with_margin.png: rotate_and_add_margin.py
	mkdir -p build
	python3 ./rotate_and_add_margin.py tag41_12_00000.png $@
	echo "Generated $@"

build/april_tag_print.pdf: april_tag_print.tex build/tag_with_margin.png
	mkdir -p build
	pdflatex -output-directory=build $<
	echo "Generated $@"

clean:
	rm -rf build

.PHONY: all clean check-deps
