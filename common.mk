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
	}'

.PHONY: setup
setup: install-uv setup-precommit ## Set up a development environment
	uv sync --frozen --all-extras

.PHONY: setup-tests
setup-tests: install-uv install-build-deps ##- Set up a testing environment without linters
	uv sync --frozen

.PHONY: setup-lint
setup-lint: install-uv install-shellcheck install-pyright install-lint-build-deps  ##- Set up a linting-only environment
	uv sync --frozen --no-install-workspace --extra lint --extra types

.PHONY: setup-docs
setup-docs: install-uv  ##- Set up a documentation-only environment
	uv sync --frozen --no-dev --no-install-workspace --extra docs

.PHONY: setup-precommit
setup-precommit: install-uv  ##- Set up pre-commit hooks in this repository.
ifeq ($(shell which pre-commit),)
	uv tool install pre-commit
endif
	pre-commit install

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
lint-codespell:  ##- Check spelling with codespell
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

.PHONY: lint-yaml
lint-yaml:  ##- Lint YAML files with yamllint
ifneq ($(CI),)
	@echo ::group::$@
endif
	uv run --extra lint yamllint .
ifneq ($(CI),)
	@echo ::endgroup::
endif

.PHONY: lint-docs
lint-docs:  ##- Lint the documentation
ifneq ($(CI),)
	@echo ::group::$@
endif
	uv run --extra docs sphinx-lint --max-line-length 88 --enable all $(DOCS)
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
	uv run coverage run --source $(PROJECT) -m pytest
	uv run coverage xml -o coverage.xml
	uv run coverage report -m
	uv run coverage html

.PHONY: docs
docs:  ## Build documentation
	uv run --extra docs sphinx-build -b html -W $(DOCS) $(DOCS)/_build

.PHONY: docs-auto
docs-auto:  ## Build and host docs with sphinx-autobuild
	uv run --extra docs sphinx-autobuild -b html --open-browser --port=8080 --watch $(PROJECT) -W $(DOCS) $(DOCS)/_build

.PHONY: pack-pip
pack-pip:  ##- Build packages for pip (sdist, wheel)
ifneq ($(CI),)
	@echo ::group::$@
endif
	uv build .
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
