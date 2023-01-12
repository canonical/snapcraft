SOURCES=setup.py snapcraft tests/*.py tests/unit
SOURCES_LEGACY=snapcraft_legacy tests/legacy

.PHONY: autoformat-black
autoformat-black:
	tox run -e format

.PHONY: freeze-requirements
freeze-requirements:
	tools/freeze-requirements.sh

.PHONY: test-black
test-black:
	tox run -e black

.PHONY: test-codespell
test-codespell:
	tox run -e codespell

.PHONY: test-flake8
test-flake8:
	tox run -e flake

.PHONY: test-isort
test-isort:
	tox run -e isort

.PHONY: test-mypy
test-mypy:
	tox run -e mypy

.PHONY: test-pydocstyle
test-pydocstyle:
	tox run -e docstyle

.PHONY: test-pylint
test-pylint:
	tox run -e pylint

.PHONY: test-pyright
test-pyright:
	tox run -e pyright

.PHONY: test-shellcheck
test-shellcheck:
	tox run -e spellcheck

.PHONY: test-legacy-units
test-legacy-units:
	tox run -e py38-withreq-legacy

.PHONY: test-units
test-units: test-legacy-units
	tox run -e py38-withreq-unit

.PHONY: tests
tests: tests-static test-units

.PHONY: tests-static
tests-static: test-black test-codespell test-flake8 test-isort test-mypy test-pydocstyle test-pyright test-pylint test-shellcheck

.PHONY: lint
lint: tests-static
