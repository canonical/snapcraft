SOURCES=setup.py snapcraft tests/*.py tests/unit
SOURCES_LEGACY=snapcraft_legacy tests/legacy

.PHONY: autoformat-black
autoformat-black:
	black $(SOURCES) $(SOURCES_LEGACY)

.PHONY: freeze-requirements
freeze-requirements:
	tools/freeze-requirements.sh

.PHONY: test-black
test-black:
	black --check --diff $(SOURCES) $(SOURCES_LEGACY)

.PHONY: test-codespell
test-codespell:
	codespell

.PHONY: test-isort
test-isort:
	isort --check $(SOURCES) $(SOURCES_LEGACY)

.PHONY: test-mypy
test-mypy:
	mypy $(SOURCES)

.PHONY: test-pydocstyle
test-pydocstyle:
	pydocstyle snapcraft

.PHONY: test-pylint
test-pylint:
	pylint snapcraft
	pylint tests/*.py tests/unit --disable=invalid-name,missing-module-docstring,missing-function-docstring,duplicate-code,protected-access,unspecified-encoding,too-many-public-methods,too-many-arguments,too-many-lines

.PHONY: test-pyright
test-pyright:
	pyright $(SOURCES)

.PHONY: test-ruff
test-ruff:
	ruff $(SOURCES) $(SOURCES_LEGACY)

.PHONY: test-shellcheck
test-shellcheck:
# Skip third-party gradlew script.
	find . \( -name .git -o -name gradlew \) -prune -o -print0 | xargs -0 file -N | grep shell.script | cut -f1 -d: | xargs shellcheck
	./tools/spread-shellcheck.py spread.yaml tests/spread/

.PHONY: test-legacy-units
test-legacy-units:
	pytest --cov-report=xml --cov=snapcraft tests/legacy/unit

.PHONY: test-units
test-units: test-legacy-units
	pytest --cov-report=xml --cov=snapcraft tests/unit

.PHONY: tests
tests: tests-static test-units

.PHONY: tests-static
tests-static: test-black test-codespell test-ruff test-isort test-mypy test-pydocstyle test-pyright test-pylint test-shellcheck

.PHONY: lint
lint: tests-static
