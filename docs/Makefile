# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?= -q
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = .
BUILDDIR      = _build
VENV          = sphinx-resources/.sphinx/venv/bin/activate


# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

install:
	@echo "... setting up virtualenv"
	python3 -m venv sphinx-resources/.sphinx/venv
	. $(VENV); pip install --upgrade -r sphinx-resources/.sphinx/requirements.txt

	@echo "\n" \
		"--------------------------------------------------------------- \n" \
		"* watch, build and serve the documentation: make run \n" \
                "* only build: make html \n" \
                "* only serve: make serve \n" \
                "* clean built doc files: make clean-doc \n" \
                "* clean full environment: make clean \n" \
		"* check spelling: make spelling \n" \
                "* check inclusive language: make woke \n" \
		"--------------------------------------------------------------- \n"
run:
	. $(VENV); sphinx-autobuild -c . -b html "$(SOURCEDIR)" "$(BUILDDIR)"

html:
	. $(VENV); $(SPHINXBUILD) -c . -b html "$(SOURCEDIR)" "$(BUILDDIR)" -w sphinx-resources/.sphinx/warnings.txt

epub:
	. $(VENV); $(SPHINXBUILD) -c . -b epub "$(SOURCEDIR)" "$(BUILDDIR)" -w sphinx-resources/.sphinx/warnings.txt

serve:
	cd "$(BUILDDIR)"; python3 -m http.server 8000

clean: clean-doc
	rm -rf sphinx-resources/.sphinx/venv

clean-doc:
	git clean -fx "$(BUILDDIR)"

spelling: html
	. $(VENV) ; python3 -m pyspelling -c sphinx-resources/.sphinx/spellingcheck.yaml

linkcheck:
	. $(VENV) ; $(SPHINXBUILD) -c . -b linkcheck  "$(SOURCEDIR)" "$(BUILDDIR)"

woke:
	type woke >/dev/null 2>&1 || { snap install woke; exit 1; }
	woke *.rst **/*.rst -c https://github.com/canonical-web-and-design/Inclusive-naming/raw/main/config.yml

.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	. $(VENV); $(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
