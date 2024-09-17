PROJECT=starcraft
SOURCES=$(wildcard *.py) $(PROJECT) tests
DOCS=docs

ifneq ($(OS),Windows_NT)
	OS := $(shell uname)
endif

.PHONY: help
help: ## Show this help.
	@printf "%-30s %s\n" "Target" "Description"
	@printf "%-30s %s\n" "------" "-----------"
	@fgrep " ## " $(MAKEFILE_LIST) | fgrep -v grep | awk -F ': .*## ' '{$$1 = sprintf("%-30s", $$1)} 1'

.PHONY: setup
setup: ## Set up a development environment
ifeq ($(OS),Linux)
	sudo snap install codespell ruff shellcheck
	sudo snap install --classic --beta astral-uv
	sudo snap alias astral-uv.uv uv
	sudo snap alias astral-uv.uvx uvx
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

.PHONY: autoformat
autoformat: format-ruff format-codespell  ## Run all automatic formatters

.PHONY: lint
lint: lint-ruff lint-codespell lint-mypy lint-pyright lint-shellcheck lint-yaml lint-docs  ## Run all linters

.PHONY: test
test: test-unit test-integration  ## Run all tests

.PHONY: docs
docs: ## Build documentation
	uv run --extra docs sphinx-build -b html -W docs docs/_build

.PHONY: docs-auto
docs-auto:  ## Build and host docs with sphinx-autobuild
	uv run --extra docs sphinx-autobuild -b html --open-browser --port=8080 --watch $(PROJECT) -W docs docs/_build

# Helpful in `help` to split the main targets from things that build
---------------- : ## ----------------

.PHONY: format-codespell
format-codespell:  ## Fix spelling issues with codespell
	uv run codespell --toml pyproject.toml --write-changes $(SOURCES)

.PHONY: format-ruff
format-ruff:  ## Automatically format with ruff
	ruff format $(SOURCES)
	ruff check --fix $(SOURCES)

.PHONY: lint-codespell
lint-codespell: ## Check spelling with codespell
	uv run codespell --toml pyproject.toml $(SOURCES)

.PHONY: lint-docs
lint-docs:  ## Lint the documentation
	uv run --extra docs sphinx-lint --max-line-length 80 --enable all $(DOCS)

.PHONY: lint-mypy
lint-mypy: ## Check types with mypy
	uv run mypy $(SOURCES)

.PHONY: lint-pyright
lint-pyright: ## Check types with pyright
	uv run pyright

.PHONY: lint-ruff
lint-ruff:  ## Lint with ruff
	ruff format --diff $(SOURCES)
	ruff check $(SOURCES)

.PHONY: lint-shellcheck
lint-shellcheck:
	sh -c 'git ls-files | file --mime-type -Nnf- | grep shellscript | cut -f1 -d: | xargs -r shellcheck'

.PHONY: lint-yaml
lint-yaml:  ## Lint YAML files with yamllint
	uv run yamllint .

.PHONY: test-unit
test-unit: ## Run unit tests
	uv run pytest --cov=$(PROJECT) --cov-config=pyproject.toml --cov-report=xml:.coverage.unit.xml --junit-xml=.results.unit.xml tests/unit

.PHONY: test-integration
test-integration:  ## Run integration tests
	uv run pytest --cov=$(PROJECT) --cov-config=pyproject.toml --cov-report=xml:.coverage.integration.xml --junit-xml=.results.integration.xml tests/integration
