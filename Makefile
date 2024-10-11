PROJECT=starcraft
SOURCES=$(wildcard *.py) $(PROJECT) tests
DOCS=docs

ifneq ($(OS),Windows_NT)
	OS := $(shell uname)
endif

.DEFAULT_GOAL := help

.ONESHELL:

.SHELLFLAGS = -ec

.PHONY: help
help: ## Show this help.
	@printf "%-41s %s\n" "Target" "Description"
	@printf "%-41s %s\n" "------" "-----------"
	@fgrep " ##" $(MAKEFILE_LIST) | fgrep -v grep | sed 's/:[^#]*/ /' | awk -F '[: ]*' \
	'{
		if ($$2 == "##")
		{
			$$1=sprintf("%-40s", $$1);
			$$2="";
			print $$0;
		}
		else
		{
			$$1=sprintf(" └%-38s", $$1);
			$$2="";
			print $$0;
		}
	}'

---------------- : ## ----------------

.PHONY: setup
setup: ## Set up a development environment
ifeq ($(OS),Linux)
	changes="`sudo snap install --no-wait codespell`"
	changes="${changes} `sudo snap install --no-wait ruff`"
	changes="${changes} `sudo snap install --no-wait shellcheck`"
	changes="${changes} `sudo snap install --classic --no-wait astral-uv`"
	for change in ${changes}; do
		snap watch ${change}
	done
else ifeq ($(OS),Windows_NT)
	pipx install uv
	choco install shellcheck
else ifeq ($(OS),Darwin)
	brew install uv
	brew install shellcheck
endif
ifneq ($(OS),Linux)
	uv tool install codespell
	uv tool install ruff
	uv tool update-shell
endif

.PHONY: setup-pre-commit
setup-precommit:  ## Set up pre-commit hooks in this repository.
ifeq (, $(shell which pre-commit))
	uv tool install pre-commit
endif
	pre-commit install

---------------- : ## ----------------

.PHONY: autoformat
autoformat: format-ruff format-codespell  ## Run all automatic formatters

.PHONY: format-ruff
format-ruff:  ##- Automatically format with ruff
	ruff check --fix $(SOURCES)
	success=true
	ruff check --fix $(SOURCES) || success=false
	ruff format $(SOURCES)
	$success || exit 1

.PHONY: format-codespell
format-codespell:  ##- Fix spelling issues with codespell
	uv run codespell --toml pyproject.toml --write-changes $(SOURCES)

---------------- : ## ----------------

.PHONY: lint
lint: lint-ruff lint-codespell lint-mypy lint-pyright lint-shellcheck lint-yaml lint-docs  ## Run all linters

.PHONY: lint-ruff
lint-ruff:  ##- Lint with ruff
	ruff check $(SOURCES)
	ruff format --diff $(SOURCES)

.PHONY: lint-codespell
lint-codespell:  ##- Check spelling with codespell
	uv run codespell --toml pyproject.toml $(SOURCES)

.PHONY: lint-mypy
lint-mypy:  ##- Check types with mypy
	uv run mypy --show-traceback --show-error-codes $(SOURCES)

.PHONY: lint-pyright
lint-pyright:  ##- Check types with pyright
	# Fix for a bug in npm
	[ -d "/home/ubuntu/.npm/_cacache" ] && chown -R 1000:1000 "/home/ubuntu/.npm" || true
	uv run pyright

.PHONY: lint-shellcheck
lint-shellcheck:  ##- Lint shell scripts
	git ls-files | file --mime-type -Nnf- | grep shellscript | cut -f1 -d: | xargs -r shellcheck

.PHONY: lint-yaml
lint-yaml:  ##- Lint YAML files with yamllint
	uv run yamllint .

.PHONY: lint-docs
lint-docs:  ##- Lint the documentation
	uv run --extra docs sphinx-lint --max-line-length 88 --enable all $(DOCS)

---------------- : ## ----------------

.PHONY: test
test: test-unit test-integration  ## Run all tests

.PHONY: test-unit
test-unit:  ##- Run unit tests
	uv run pytest --cov=$(PROJECT) --cov-config=pyproject.toml --cov-report=xml:.coverage.unit.xml --junit-xml=.results.unit.xml tests/unit

.PHONY: test-integration
test-integration:  ##- Run integration tests
	uv run pytest --cov=$(PROJECT) --cov-config=pyproject.toml --cov-report=xml:.coverage.integration.xml --junit-xml=.results.integration.xml tests/integration

---------------- : ## ----------------

.PHONY: coverage
coverage:  ## Generate coverage report
	coverage run --source starcraft -m pytest
	coverage xml -o coverage.xml
	coverage report -m
	coverage html

---------------- : ## ----------------

.PHONY: docs
docs:  ## Build documentation
	uv run --extra docs sphinx-build -b html -W docs docs/_build

.PHONY: docs-auto
docs-auto:  ## Build and host docs with sphinx-autobuild
	uv run --extra docs sphinx-autobuild -b html --open-browser --port=8080 --watch $(PROJECT) -W docs docs/_build
