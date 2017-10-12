# This makefile is only to make pdfs
###### YOU SHOULD NOT CHANGE BELOW THIS LINE ######

PDFS:=linux.c linux.h \
	win32.c win32.dll.c win32.h win32.rc win32.def

ENSCRIPT:=-MA4 --color -f Courier8 -C --margins=15:15:0:20
ifneq ($(strip $(wildcard /usr/share/enscript/mine-web.hdr)),)
  ENSCRIPT+= --fancy-header=mine-web
else
  ENSCRIPT+= --fancy-header=a2ps
endif

PDFS:=${sort ${PDFS}}

.PHONY: FORCE
.PHONY: all cleanpdfs pdfs

# all - do everything (default) #
all: pdfs
	@echo "*** Done! ***"

# new - clean and do everything again #
new: cleanpdfs all


# cleanpdfs - remove any compiled files and PDFs #
cleanpdfs:
	rm -f ./pdf/*.pdf


# pdfs - generate PDFs for each source file #
ifneq ($(strip $(wildcard /usr/bin/ps2pdf)),)
ifneq ($(strip $(wildcard /usr/bin/enscript)),)
pdfs: ./pdf/ ${addprefix ./pdf/,${addsuffix .pdf,${PDFS}}}

./pdf/:
	mkdir ./pdf/

./pdf/%.pdf: %
	enscript ${ENSCRIPT} -Ec $< -p - | ps2pdf - $@

./pdf/%.pdf:
	@echo "*** Cannot make $@ - '$*' does not exist ***"
else
pdfs:
	@echo "WARNING: enscript is not installed - cannot generate PDF files"
endif
else
pdfs:
	@echo "WARNING: ps2pdf is not installed - cannot generate PDF files"
endif
