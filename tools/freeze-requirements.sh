#!/bin/bash -eux

requirements_fixups() {
  req_file="$1"

  sed -i '/python-apt=*/d' "$req_file"
  echo 'python-apt @ http://archive.ubuntu.com/ubuntu/pool/main/p/python-apt/python-apt_1.6.5ubuntu0.5.tar.xz; sys.platform == "linux"' >> "$req_file"
  echo 'python-distutils-extra @ https://launchpad.net/python-distutils-extra/trunk/2.39/+download/python-distutils-extra-2.39.tar.gz; sys_platform == "linux"' >> "$req_file"

  sed -i '/PyNaCl=*/d' "$req_file"
  echo 'PyNaCl==1.3.0; sys.platform != "linux"' >> "$req_file"
  echo 'PyNaCl @ https://files.pythonhosted.org/packages/61/ab/2ac6dea8489fa713e2b4c6c5b549cc962dd4a842b5998d9e80cf8440b7cd/PyNaCl-1.3.0.tar.gz; sys.platform == "linux"' >> "$req_file"

  sed -i '/pkg-resources==0.0.0/d' "$req_file"
}

venv_dir="$(mktemp -d)"

# Enable system-site-packages to find python3-apt.
python3 -m venv "$venv_dir"

# shellcheck disable=SC1090
source "$venv_dir/bin/activate"

pip install -U setuptools pip wheel

# Pull in host python3-apt site package to avoid installation.
site_pkgs="$(readlink -f "$venv_dir"/lib/python3.*/site-packages/)"
cp -r /usr/lib/python3/dist-packages/{apt,python_apt}* "$site_pkgs"

pip install -e .
pip freeze --exclude-editable > requirements.txt
requirements_fixups "requirements.txt"

# Set the configured python-apt and python-distutils-extra packages.
pip install -e .[dev]
pip freeze --exclude-editable > requirements-devel.txt
requirements_fixups "requirements-devel.txt"

rm -rf "$venv_dir"
