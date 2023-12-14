#!/bin/bash -eux

requirements_fixups() {
  req_file="$1"

  # Python apt library pinned to source.
  sed -i '/python-apt=*/d' "$req_file"
  echo 'python-apt @ https://launchpad.net/ubuntu/+archive/primary/+sourcefiles/python-apt/2.4.0ubuntu1/python-apt_2.4.0ubuntu1.tar.xz; sys.platform == "linux"' >> "$req_file"

  # https://bugs.launchpad.net/ubuntu/+source/python-pip/+bug/1635463
  sed -i '/pkg[-_]resources==0.0.0/d' "$req_file"

  # Pinned pyinstaller for windows.
  if [[ "$req_file" == "requirements-devel.txt" ]]; then
      sed -i '/pyinstaller/d' "$req_file"
      echo 'pyinstaller==4.10; sys.platform == "win32"' >> "$req_file"
  fi
}

venv_dir="$(mktemp -d)"

# Enable system-site-packages to find python3-apt.
python3 -m venv "$venv_dir"

# shellcheck disable=SC1090,SC1091
source "$venv_dir/bin/activate"

pip install -U setuptools pip wheel

# Pull in host python3-apt site package to avoid installation.
site_pkgs="$(readlink -f "$venv_dir"/lib/python3.*/site-packages/)"
temp_dir="$(mktemp -d)"
pushd "$temp_dir"
apt download python3-apt
dpkg -x ./*.deb .
cp -r usr/lib/python3/dist-packages/* "$site_pkgs"
popd

pip install -e .
pip freeze --exclude-editable > requirements.txt
requirements_fixups "requirements.txt"

# Set the configured python-apt and python-distutils-extra packages.
pip install -e .[dev]
pip freeze --exclude-editable > requirements-devel.txt
requirements_fixups "requirements-devel.txt"

rm -rf "$venv_dir"
