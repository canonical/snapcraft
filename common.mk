# Common items for all Starcraft Makefiles. Should only be edited in the `starbase` repository:
# https://github.com/canonical/starbase

SOURCES=$(wildcard *.py) $(PROJECT) tests

# Env vars for the docs Sphinx Stack. They must be exported so make can pass them to the
# docs Makefile.
export DOCS_BUILDDIR ?= _build
export DOCS_VENVDIR ?= ../.venv
export VALE_DIR ?= $(DOCS_VENVDIR)/lib/python*/site-packages/vale
export SPHINX_AUTOBUILD_OPTS ?= --ignore "$(DOCS_VENVDIR)/*" --ignore "reference/commands/*" -D=llms_txt_enabled=0

ifneq ($(OS),Windows_NT)
	OS := $(shell uname)
endif
ifdef CI
	APT := apt-get --yes
else
	APT := apt-get
endif

PRETTIER=npm exec --package=prettier@3.6.0 -- prettier --log-level warn # renovate: datasource=npm
PRETTIER_FILES="**/*.{yaml,yml,json,json5,css,md}"

# Cutoff (in seconds) before a test is considered slow by pytest
SLOW_CUTOFF_TIME ?= 1

# By default we should not update the uv lock file here.
export UV_FROZEN := true

.DEFAULT_GOAL := help

.ONESHELL:

.SHELLFLAGS = -ec

.PHONY: help
help: ## Show this help.
	@printf "\033[1m%-30s\033[0m | \033[1m%s\033[0m\n" "Target" "Description"
	@printf "\033[2m%-30s + %-41s\033[0m\n" "------------------------------" "------------------------------------------------"
	@cat $$(echo $(MAKEFILE_LIST) | tac --separator=' ' 2>/dev/null || echo $(MAKEFILE_LIST)) | grep -E '^[^[:space:]][^:]*\:[^#]*##' | \
	sed -e 's/:[^#]*/ /' | sort -V | \
	awk -F '[: ]+' '{ if ($$2 == "##") { $$1=sprintf(" %-28s", $$1); $$2=" | "; print $$0; } else { $$1=sprintf("  └ %-25s", $$1); $$2=" | "; $$3=sprintf(" └ %s", $$3); print $$0; } }' | \
	uniq

.PHONY: setup
setup: install-uv _setup-docs _setup-lint _setup-tests setup-precommit install-build-deps  ## Set up the development environment
	uv sync $(UV_TEST_GROUPS) $(UV_LINT_GROUPS) $(UV_DOCS_GROUPS)

.PHONY: setup-docs
setup-docs: _setup-docs  ##- Set up the documentation environment
ifneq ($(CI),)
	@echo ::group::$@
endif
	uv sync --no-dev $(UV_DOCS_GROUPS)
ifneq ($(CI),)
	@echo ::endgroup::
endif

.PHONY: _setup-docs
_setup-docs: install-uv

.PHONY: setup-lint
setup-lint: _setup-lint  ##- Set up a linting-only environment
	uv sync $(UV_LINT_GROUPS)

.PHONY: _setup-lint
_setup-lint: install-uv install-shellcheck install-shfmt install-pyright install-lint-build-deps install-actionlint

.PHONY: setup-tests
setup-tests: _setup-tests ##- Set up a testing environment without linters
	uv sync $(UV_TEST_GROUPS)

.PHONY: _setup-tests
_setup-tests: install-uv install-build-deps

.PHONY: setup-tics
setup-tics: install-uv install-build-deps ##- Set up a testing environment for Tiobe TICS
	uv venv
	uv sync $(UV_TEST_GROUPS) $(UV_LINT_GROUPS) $(UV_TICS_GROUPS)
ifneq ($(CI),)
	echo $(PWD)/.venv/bin >> $(GITHUB_PATH)
endif

.PHONY: setup-precommit
setup-precommit: install-uv  ##- Set up pre-commit hooks in this repository.
ifeq ($(shell which pre-commit),)
	uv tool run pre-commit install
else
	pre-commit install
endif

.PHONY: clean
clean:  ## Clean up the development environment
	uv tool run pyclean .
	rm -rf dist build docs/_build docs/_linkcheck docs/reference/gen *.snap .coverage* .venv

.PHONY: autoformat
autoformat: format  # Hidden alias for 'format'

.PHONY: format-ruff
format-ruff: install-ruff  ##- Automatically format with ruff
	success=true
	ruff check --fix $(SOURCES) || success=false
	ruff format $(SOURCES)
	$$success || exit 1

.PHONY: format-codespell
format-codespell:  ##- Fix spelling issues with codespell
	uv run codespell --toml pyproject.toml --write-changes $(SOURCES)

.PHONY: format-pre-commit
format-pre-commit:  ##- Format the entire repository using pre-commit
	uv tool run pre-commit run

.PHONY: format-prettier
format-prettier: install-npm  ##- Format files with prettier
	$(PRETTIER) --write $(PRETTIER_FILES)

.PHONY: format-shfmt
format-shfmt: install-shfmt ##- Format shell scripts
	@# jinja2 shell script templates are mistakenly counted as "true" shell scripts due to their shebang,
	@# so explicitly filter them out
	git ls-files -z | xargs -0 sh -c 'for f; do case "$$f" in *.sh.j2) continue;; esac; file --mime-type -Nn -- "$$f" | grep -q shellscript && printf "%s\0" "$$f"; done' -- | xargs -0r shfmt -w

.PHONY: lint-ruff
lint-ruff: install-ruff  ##- Lint with ruff
ifneq ($(CI),)
	@echo ::group::$@
endif
	ruff check $(SOURCES)
	ruff format --diff $(SOURCES)
ifneq ($(CI),)
	@echo ::endgroup::
endif

.PHONY: lint-codespell
lint-codespell: install-codespell  ##- Check spelling with codespell
ifneq ($(CI),)
	@echo ::group::$@
endif
	uv run codespell --toml pyproject.toml $(SOURCES)
ifneq ($(CI),)
	@echo ::endgroup::
endif

.PHONY: lint-mypy
lint-mypy:  ##- Check types with mypy
ifneq ($(CI),)
	@echo ::group::$@
endif
	uv run mypy --show-traceback --show-error-codes $(PROJECT)
ifneq ($(CI),)
	@echo ::endgroup::
endif

.PHONY: lint-pyright
lint-pyright:  ##- Check types with pyright
ifneq ($(CI),)
	@echo ::group::$@
endif
ifneq ($(shell which pyright),) # Prefer the system pyright
	pyright --pythonpath .venv/bin/python
else
	uv tool run pyright --pythonpath .venv/bin/python
endif
ifneq ($(CI),)
	@echo ::endgroup::
endif

.PHONY: lint-ty
lint-ty: install-ty  ##- Check types with Astral ty
ifneq ($(CI),)
	@echo ::group::$@
endif
	ty check --python .venv $(SOURCES)
ifneq ($(CI),)
	@echo ::endgroup::
endif

.PHONY: lint-uv-lockfile
lint-uv-lockfile: install-uv  ##- Check that uv.lock matches expectations from pyproject.toml
	unset UV_FROZEN
	uv lock --check

.PHONY: lint-shfmt
lint-shfmt: install-shfmt  ##- Lint shell script formatting
ifneq ($(CI),)
	@echo ::group::$@
endif
	@# jinja2 shell script templates are mistakenly counted as "true" shell scripts due to their shebang,
	@# so explicitly filter them out
	git ls-files -z | xargs -0 sh -c 'for f; do case "$$f" in *.sh.j2) continue;; esac; file --mime-type -Nn -- "$$f" | grep -q shellscript && printf "%s\0" "$$f"; done' -- | xargs -0r shfmt --diff
ifneq ($(CI),)
	@echo ::endgroup::
endif

.PHONY: lint-shellcheck
lint-shellcheck:  ##- Lint shell scripts
ifneq ($(CI),)
	@echo ::group::$@
endif
	@# jinja2 shell script templates are mistakenly counted as "true" shell scripts due to their shebang,
	@# so explicitly filter them out
	git ls-files -z | xargs -0 sh -c 'for f; do case "$$f" in *.sh.j2) continue;; esac; file --mime-type -Nn -- "$$f" | grep -q shellscript && printf "%s\0" "$$f"; done' -- | xargs -0r shellcheck
ifneq ($(CI),)
	@echo ::endgroup::
endif


.PHONY: lint-prettier
lint-prettier: install-npm  ##- Lint files with prettier
ifneq ($(CI),)
	@echo ::group::$@
endif
	$(PRETTIER) --check $(PRETTIER_FILES)
ifneq ($(CI),)
	@echo ::endgroup::
endif

.PHONY: lint-actions
lint-actions: install-actionlint  ##- Lint GitHub actions with actionlint
ifneq ($(CI),)
	@echo ::group::$@
endif
	actionlint
ifneq ($(CI),)
	@echo ::endgroup::
endif

# Legacy alias for linting docs
.PHONY: lint-docs
lint-docs: docs-lint  ##- Lint the documentation

.PHONY: lint-twine
lint-twine: pack-pip  ##- Lint Python packages with twine
ifneq ($(CI),)
	@echo ::group::$@
endif
	uv tool run twine check dist/*
ifneq ($(CI),)
	@echo ::endgroup::
endif

.PHONY: test
test:  ## Run all tests
	uv run pytest

.PHONY: test-fast
test-fast:  ##- Run fast tests
	uv run pytest -m 'not slow'

.PHONY: test-slow
test-slow:  ##- Run slow tests
	uv run pytest -m 'slow'

.PHONY: test-coverage
test-coverage:  ##- Generate coverage report
ifeq ($(COVERAGE_SOURCE),)
	uv run coverage run --source $(PROJECT),tests -m pytest
else
	uv run coverage run --source $(COVERAGE_SOURCE),tests -m pytest
endif
	uv run coverage xml -o results/coverage.xml
	# for backwards compatibility
	# https://github.com/canonical/starflow/blob/3447d302cb7883cbb966ce0ec7e5b3dfd4bb3019/.github/workflows/test-python.yaml#L109
	cp results/coverage.xml coverage.xml
	uv run coverage report -m
	uv run coverage html

.PHONY: test-find-slow
test-find-slow:  ##- Identify slow tests. Set cutoff time in seconds with SLOW_CUTOFF_TIME
	uv run pytest --durations 0 --durations-min $(SLOW_CUTOFF_TIME)

# Alias for `html` target in docs project. We want to use our own `.venv`, so we
# replace it.
.PHONY: docs
docs:  ## Render the documentation to disk
ifneq ($(CI),)
	@echo ::group::$@
endif
	$(MAKE) docs-install
	$(MAKE) -C docs html --no-print-directory
ifneq ($(CI),)
	@echo ::endgroup::
endif

# Alias for `serve` target in docs project
.PHONY: docs-auto
docs-auto:  ##- Render the documentation in a live session
	$(MAKE) docs-install
	$(MAKE) -C docs run --no-print-directory

# Override for `install` target in docs project. We still need the Vale setup, so we
# run that after the parent docs setup.
.PHONY: docs-install
docs-install: _setup-docs  ##- Set up documentation packages
ifneq ($(CI),)
ifeq ($(MAKELEVEL),0)
	@echo ::group::$@
endif
endif
	$(MAKE) -C docs vale-install --no-print-directory
ifneq ($(CI),)
ifeq ($(MAKELEVEL),0)
	@echo ::endgroup::
endif
endif

# Alias for `setup-docs`
.PHONY: docs-setup
docs-setup: setup-docs

# Override for `clean` target in docs project. We don't want to touch `.venv`.
.PHONY: docs-clean
docs-clean:  ##- Clean the temporary files used in documentation
	$(MAKE) -C docs clean-doc --no-print-directory
	rm -rf docs/_dev/node_modules/
	rm -rf docs/_dev/styles
	rm -f docs/_dev/vale.ini

# Override for `help` target in docs project
.PHONY: docs-help
docs-help:  ##- List the individual commands in the documentation subproject.
	@echo "Commands in the documentation subproject:"
	$(MAKE) -C docs help --no-print-directory
	@echo "Run these commands from inside the 'docs/' directory."

# Override for `pymarkdownlnt-install` target in docs project. Make it a noop.
.PHONY: docs-pymarkdownlnt-install
docs-pymarkdownlnt-install:
	@echo "Cannot run 'docs-pymarkdownlnt'. This project doesn't use Markdown."

# Override for `lint-md` target in docs project. Make it a noop.
.PHONY: docs-lint-md
docs-lint-md:
	@echo "Cannot run 'docs-lint-md'. This project doesn't use Markdown."

# Passthrough for the rest of the targets in docs project
.PHONY: docs-%
docs-%:
ifneq ($(CI),)
	@echo ::group::$@
endif
	$(MAKE) docs-install
	$(MAKE) -C docs $(@:docs-%=%) --no-print-directory
ifneq ($(CI),)
	@echo ::endgroup::
endif

# Run our own docs linting, then pass to the docs
.PHONY: docs-lint
docs-lint:  ##- Lint the documentation
ifneq ($(CI),)
	@echo ::group::$@
endif
	$(MAKE) docs-install
	uv run $(UV_DOCS_GROUPS) sphinx-lint docs \
	--ignore docs/_dev \
	--ignore docs/_build \
	--ignore docs/reference/commands \
	--enable all \
	-d line-too-long,missing-underscore-after-hyperlink,missing-space-in-hyperlink
	$(MAKE) -C docs spelling --no-print-directory
	$(MAKE) -C docs woke --no-print-directory
	$(MAKE) -C docs linkcheck --no-print-directory
ifneq ($(CI),)
	@echo ::endgroup::
endif

.PHONY: pack-pip
pack-pip:  ##- Build packages for pip (sdist, wheel)
ifneq ($(CI),)
	@echo ::group::$@
endif
	uv build --quiet .
ifneq ($(CI),)
	@echo ::endgroup::
endif

# Below are intermediate targets for setup. They are not included in help as they should
# not be used independently.

.PHONY: install-uv
install-uv:
ifneq ($(shell which uv),)
else ifneq ($(shell which snap),)
	sudo snap install --classic astral-uv
else ifneq ($(shell which brew),)
	brew install uv
else ifeq ($(OS),Windows_NT)
	pwsh -c "irm https://astral.sh/uv/install.ps1 | iex"
else
	curl -LsSf https://astral.sh/uv/install.sh | sh
endif

.PHONY: install-actionlint
install-actionlint:
ifneq ($(shell which actionlint),)
else ifneq ($(shell which snap),)
	sudo snap install actionlint
else ifneq ($(shell which brew),)
	brew install actionlint
else
	$(warning Actionlint not installed. Please install it yourself.)
endif

.PHONY: install-codespell
install-codespell:
ifneq ($(shell which codespell),)
else ifneq ($(shell which snap),)
	sudo snap install codespell
else ifneq ($(shell which brew),)
	make install-uv
	uv tool install codespell
else
	$(warning Codespell not installed. Please install it yourself.)
endif

.PHONY: install-pyright
install-pyright: install-uv
ifneq ($(shell which pyright),)
else ifneq ($(shell which snap),)
	sudo snap install --classic pyright
else
	# Workaround for a bug in npm
	[ -d "$(HOME)/.npm/_cacache" ] && chown -R `id -u`:`id -g` "$(HOME)/.npm" || true
	uv tool install pyright
endif

.PHONY: install-ruff
install-ruff:
ifneq ($(shell which ruff),)
else ifneq ($(shell which snap),)
	sudo snap install ruff
else
	make install-uv
	uv tool install ruff
endif

.PHONY: install-shellcheck
install-shellcheck:
ifneq ($(shell which shellcheck),)
else ifneq ($(shell which snap),)
	sudo snap install shellcheck
else ifneq ($(shell which brew),)
	brew install shellcheck
else
	$(warning Shellcheck not installed. Please install it yourself.)
endif

.PHONY: install-shfmt
install-shfmt:
ifneq ($(shell which shfmt),)
else ifneq ($(shell which snap),)
	sudo snap install shfmt
else ifneq ($(shell which brew),)
	brew install shfmt
else
	$(warning shfmt not installed. Please install it yourself.)
endif

.PHONY: install-ty
install-ty:
ifneq ($(shell which ty),)
else ifneq ($(shell which snap),)
	sudo snap install --beta astral-ty
	sudo snap alias astral-ty.ty ty
else
	make install-uv
	uv tool install ty
endif

.PHONY: install-npm
install-npm:
ifneq ($(shell which npm),)
else ifneq ($(shell which snap),)
	sudo snap install --classic node
else ifneq ($(shell which brew),)
	brew install node
else
	$(error npm not installed. Please install it yourself.)
endif
