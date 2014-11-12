SOURCES = \
	inverse-kinematics \
	optimal-control \
	homogeneous-matrix \
	solution1 \
	solution2 \
	solution3 \
	solution4 \
	solution9 \
	solution11

HTML_FILES=$(SOURCES:%=html/%.html)

all: $(HTML_FILES)

html/%.html: %
	asciidoc --filter latex -a toc -a icons -o $@ $<

clean:
	rm -f $(HTML_FILES)
