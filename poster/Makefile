ALL: poster.pdf 

%.pdf: %.tex 
	pdflatex $< &&	pdflatex $< && pdflatex $<

clean:
	  rm -f *.aux	*.bbl	*.blg	*.log	poster.pdf
