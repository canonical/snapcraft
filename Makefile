.PHONY: autoformat-black
autoformat-black:
	black .

.PHONY: test-black
test-black:
	black --check --diff .

.PHONY: test-codespell
test-codespell:
	codespell --quiet-level 4 --ignore-words-list keyserver --skip '*.tar,*.xz,*.zip,*.bz2,*.7z,*.gz,*.deb,*.rpm,*.snap,*.gpg,*.pyc,*.png,*.ico,*.jar,changelog,.git,.hg,.mypy_cache,.tox,.venv,_build,buck-out,__pycache__,build,dist,.vscode,parts,stage,prime,test_appstream.py,./snapcraft.spec,./.direnv'

.PHONY: test-flake8
test-flake8:
	python3 -m flake8 .

.PHONY: test-mypy
test-mypy:
	mypy .

.PHONY: test-shellcheck
test-shellcheck:
# Skip third-party gradlew script.
	find . \( -name .git -o -name gradlew \) -prune -o -print0 | xargs -0 file -N | grep shell.script | cut -f1 -d: | xargs shellcheck
	./tools/spread-shellcheck.py spread.yaml tests/spread/

.PHONY: test-units
test-units:
	pytest --cov-report=xml --cov=snapcraft tests/unit

.PHONY: tests
tests: tests-static test-units

.PHONY: tests-static
tests-static: test-black test-codespell test-flake8 test-mypy test-shellcheck
