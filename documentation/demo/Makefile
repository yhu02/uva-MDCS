.PHONY: clean

DOC = maze_turtle_demo_notes.pdf

all: $(DOC)

%.pdf: %.md
	pandoc --output $@ --template eisvogel $<

clean:
	rm -f $(DOC)
