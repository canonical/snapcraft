# Common items for all Starcraft Makefiles. Should only be edited in the `starbase` repository:
# https://github.com/canonical/starbase

SOURCES=$(wildcard *.py) $(PROJECT) tests
DOCS=docs

ifneq ($(OS),Windows_NT)
	OS := $(shell uname)
endif
ifdef CI
	APT := apt-get --yes
else
	APT := apt-get
endif

PRETTIER=npm exec --package=prettier -- prettier --log-level warn
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
	@printf "\e[1m%-30s\e[0m | \e[1m%s\e[0m\n" "Target" "Description"
	printf "\e[2m%-30s + %-41s\e[0m\n" "------------------------------" "------------------------------------------------"
	egrep '^[^:]+\: [^#]*##' $$(echo $(MAKEFILE_LIST) | tac --separator=' ') | sed -e 's/^[^:]*://' -e 's/:[^#]*/ /' | sort -V| awk -F '[: ]*' \
	'{
		if ($$2 == "##")
		{
			$$1=sprintf(" %-28s", $$1);
			$$2=" | ";
			print $$0;
		}
		else
		{
			$$1=sprintf("  └ %-25s", $$1);
			$$2=" | ";
			$$3=sprintf(" └ %s", $$3);
			print $$0;
		}
	}' | uniq

.PHONY: setup
setup: install-uv setup-precommit install-build-deps ## Set up a development environment
	uv sync $(UV_TEST_GROUPS) $(UV_LINT_GROUPS) $(UV_DOCS_GROUPS)

.PHONY: setup-tests
setup-tests: install-uv install-build-deps ##- Set up a testing environment without linters
	uv sync $(UV_TEST_GROUPS)

.PHONY: setup-tics
setup-tics: install-uv install-build-deps ##- Set up a testing environment for Tiobe TICS
	uv venv
	uv sync $(UV_TEST_GROUPS) $(UV_LINT_GROUPS) $(UV_TICS_GROUPS)
ifneq ($(CI),)
	echo $(PWD)/.venv/bin >> $(GITHUB_PATH)
endif

.PHONY: setup-lint
setup-lint: install-uv install-shellcheck install-pyright install-lint-build-deps  ##- Set up a linting-only environment
	uv sync $(UV_LINT_GROUPS)

.PHONY: setup-docs
setup-docs: install-uv  ##- Set up a documentation-only environment
	uv sync --no-dev $(UV_DOCS_GROUPS)

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
	rm -rf dist/ build/ docs/_build/ *.snap .coverage*

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

.PHONY: format-prettier
format-prettier: install-npm  ##- Format files with prettier
	$(PRETTIER) --write $(PRETTIER_FILES)

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

.PHONY: lint-shellcheck
lint-shellcheck:  ##- Lint shell scripts
ifneq ($(CI),)
	@echo ::group::$@
endif
	git ls-files | file --mime-type -Nnf- | grep shellscript | cut -f1 -d: | xargs -r shellcheck
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

.PHONY: lint-docs
lint-docs:  ##- Lint the documentation
ifneq ($(CI),)
	@echo ::group::$@
endif
	uv run $(UV_DOCS_GROUPS) sphinx-lint --max-line-length 88 --ignore docs/reference/commands --ignore docs/_build --enable all $(DOCS) -d missing-underscore-after-hyperlink,missing-space-in-hyperlink
ifneq ($(CI),)
	@echo ::endgroup::
endif

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
test-coverage:  ## Generate coverage report
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

.PHONY: docs
docs:  ## Build documentation
	uv run $(UV_DOCS_GROUPS) sphinx-build -b dirhtml -W $(DOCS) $(DOCS)/_build

.PHONY: docs-auto
docs-auto:  ## Build and host docs with sphinx-autobuild
	uv run --group docs sphinx-autobuild -b dirhtml --open-browser --port=8080 --watch $(PROJECT) -W $(DOCS) $(DOCS)/_build

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
