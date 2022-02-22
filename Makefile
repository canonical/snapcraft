SOURCES=setup.py snapcraft tests/*.py tests/unit

.PHONY: autoformat-black
autoformat-black:
	black $(SOURCES)

.PHONY: freeze-requirements
freeze-requirements:
	tools/freeze-requirements.sh

.PHONY: test-black
test-black:
	black --check --diff $(SOURCES)

.PHONY: test-codespell
test-codespell:
	codespell --quiet-level 4 --ignore-words-list crate,keyserver --skip '*.tar,*.xz,*.zip,*.bz2,*.7z,*.gz,*.deb,*.rpm,*.snap,*.gpg,*.pyc,*.png,*.ico,*.jar,*.so,changelog,.git,.hg,.mypy_cache,.tox,.venv,_build,buck-out,__pycache__,build,dist,.vscode,parts,stage,prime,test_appstream.py,./snapcraft.spec,./.direnv,./.pytest_cache'

.PHONY: test-flake8
test-flake8:
	python3 -m flake8 $(SOURCES)

.PHONY: test-isort
test-isort:
	isort --check $(SOURCES)

.PHONY: test-mypy
test-mypy:
	mypy $(SOURCES)

.PHONY: test-pydocstyle
test-pydocstyle:
	pydocstyle snapcraft

.PHONY: test-pylint
test-pylint:
	pylint snapcraft
	pylint tests/*.py tests/unit --disable=invalid-name,missing-module-docstring,missing-function-docstring,no-self-use,duplicate-code,protected-access,unspecified-encoding

.PHONY: test-pyright
test-pyright:
	pyright $(SOURCES)

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
tests-static: test-black test-codespell test-flake8 test-isort test-mypy test-pydocstyle test-pyright test-pylint test-shellcheck

.PHONY: lint
lint: tests-static
