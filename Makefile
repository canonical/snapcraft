PROJECT=snapcraft
# Define when more than the main package tree requires coverage
# like is the case for snapcraft (snapcraft and snapcraft_legacy):
COVERAGE_SOURCE="snapcraft,snapcraft_legacy"
UV_TEST_GROUPS := "--group=dev"
UV_DOCS_GROUPS := "--group=docs"
UV_LINT_GROUPS := "--group=lint" "--group=types"
UV_TICS_GROUPS := "--group=tics"

ifneq ($(wildcard /etc/os-release),)
include /etc/os-release
endif
ifdef VERSION_CODENAME
UV_TEST_GROUPS += "--group=dev-$(VERSION_CODENAME)"
UV_DOCS_GROUPS += "--group=dev-$(VERSION_CODENAME)"
UV_LINT_GROUPS += "--group=dev-$(VERSION_CODENAME)"
UV_TICS_GROUPS += "--group=dev-$(VERSION_CODENAME)"
endif

include common.mk

.PHONY: format
format: format-ruff format-codespell format-prettier  ## Run all automatic formatters

# Override the common.mk lint-docs target until https://github.com/canonical/snapcraft/issues/5229 is resolved
.PHONY: lint-docs
lint-docs:  ##- Lint the documentation
ifneq ($(CI),)
	@echo ::group::$@
endif
	uv run $(UV_DOCS_GROUPS) sphinx-lint --ignore docs/reference/commands --ignore docs/_build --enable all $(DOCS) -d missing-underscore-after-hyperlink,missing-space-in-hyperlink,line-too-long
ifneq ($(CI),)
	@echo ::endgroup::
endif

.PHONY: lint
lint: lint-ruff lint-codespell lint-mypy lint-prettier lint-pyright lint-shellcheck lint-docs lint-twine  ## Run all linters

.PHONY: pack
pack: pack-pip  ## Build all packages

.PHONY: pack-snap
pack-snap: snap/snapcraft.yaml  ##- Build snap package
ifeq ($(shell which snapcraft),)
	sudo snap install --classic snapcraft
endif
	snapcraft pack

.PHONY: publish
publish: publish-pypi  ## Publish packages

.PHONY: publish-pypi
publish-pypi: clean package-pip lint-twine  ##- Publish Python packages to pypi
	uv tool run twine upload dist/*

# Find dependencies that need installing
APT_PACKAGES :=
ifeq ($(wildcard /usr/include/libxml2/libxml/xpath.h),)
APT_PACKAGES += libxml2-dev
endif
ifeq ($(wildcard /usr/include/libxslt/xslt.h),)
APT_PACKAGES += libxslt1-dev
endif
ifeq ($(wildcard /usr/share/doc/python3-venv/copyright),)
APT_PACKAGES += python3-venv
endif
ifeq ($(wildcard /usr/share/doc/libapt-pkg-dev/copyright),)
APT_PACKAGES += libapt-pkg-dev
endif
ifeq ($(wildcard /usr/share/doc/libgit2-dev/copyright),)
APT_PACKAGES += libgit2-dev
endif
ifeq ($(shell which cargo),)
APT_PACKAGES += cargo
endif
ifeq ($(wildcard /usr/share/doc/python3-dev),)
APT_PACKAGES += python3-dev
endif
ifeq ($(wildcard /usr/share/doc/libffi8),)
APT_PACKAGES += libffi-dev
endif
ifeq ($(wildcard /usr/share/doc/pkg-config/copyright),)
APT_PACKAGES += pkg-config
endif
ifeq ($(wildcard /usr/share/doc/libssl-dev/copyright),)
APT_PACKAGES += libssl-dev
endif
ifeq ($(wildcard /usr/share/doc/libyaml-dev/copyright),)
APT_PACKAGES += libyaml-dev
endif
# Needed for xdelta3 tests
ifeq ($(wildcard /usr/share/doc/xdelta3/copyright),)
APT_PACKAGES += xdelta3
endif
ifeq ($(wildcard /usr/share/doc/patchelf/copyright),)
APT_PACKAGES += patchelf
endif

# Used for installing build dependencies in CI.
.PHONY: install-build-deps
install-build-deps: install-lint-build-deps
ifeq ($(APT_PACKAGES),)
else ifeq ($(shell which apt-get),)
	$(warning Cannot install build dependencies without apt.)
	$(warning Please ensure the equivalents to these packages are installed: $(APT_PACKAGES))
else
	sudo $(APT) install $(APT_PACKAGES)
endif

# If additional build dependencies need installing in order to build the linting env.
.PHONY: install-lint-build-deps
install-lint-build-deps:
