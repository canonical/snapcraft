#!/bin/sh

deactivate() {
    unset UBUNTU_STORE_API_ROOT_URL
    unset UBUNTU_STORE_SEARCH_ROOT_URL
    unset UBUNTU_STORE_UPLOAD_ROOT_URL
    unset UBUNTU_SSO_API_ROOT_URL
    unset TEST_STORE
    export PS1="$ORIGINAL_PS1"
    unset ORIGINAL_PS1
    unset deactivate
}

export UBUNTU_STORE_API_ROOT_URL="https://dashboard.staging.snapcraft.io/dev/api/"
export UBUNTU_STORE_SEARCH_ROOT_URL="https://api.staging.snapcraft.io/"
export UBUNTU_STORE_UPLOAD_ROOT_URL="https://upload.apps.staging.ubuntu.com/"
export UBUNTU_SSO_API_ROOT_URL="https://login.staging.ubuntu.com/api/v2/"
export TEST_STORE="staging"

export ORIGINAL_PS1="$PS1"
export PS1="$PS1 snapcraft staging servers> "
