SOURCES=setup.py snapcraft tests/*.py tests/unit
SOURCES_LEGACY=snapcraft_legacy tests/legacy

.PHONY: autoformat-black
autoformat-black:
	tox run -e format-black

.PHONY: freeze-requirements
freeze-requirements:
	tools/freeze-requirements.sh

.PHONY: test-black
test-black:
	tox run -e lint-black

.PHONY: test-codespell
test-codespell:
	tox run -e lint-codespell

.PHONY: test-isort
test-isort:
	tox run -e lint-isort

.PHONY: test-mypy
test-mypy:
	tox run -e lint-mypy

.PHONY: test-pydocstyle
test-pydocstyle:
	tox run -e lint-docstyle

.PHONY: test-pylint
test-pylint:
	tox run -e lint-pylint

.PHONY: test-pyright
test-pyright:
	tox run -e lint-pyright

.PHONY: test-ruff
test-ruff:
	ruff --config snapcraft_legacy/ruff.toml $(SOURCES_LEGACY)
	ruff $(SOURCES)

.PHONY: test-shellcheck
test-shellcheck:
	tox run -e lint-shellcheck

.PHONY: test-legacy-units
test-legacy-units:
	tox run -e test-legacy-py310

.PHONY: test-units
test-units: test-legacy-units
	tox run -e test-py310

.PHONY: tests
tests: tests-static test-units

.PHONY: tests-static
tests-static: test-black test-codespell test-ruff test-isort test-mypy test-pydocstyle test-pyright test-pylint test-shellcheck

.PHONY: lint
lint: tests-static
