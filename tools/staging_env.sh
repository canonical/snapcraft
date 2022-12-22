#!/bin/sh

deactivate() {
    unset STORE_DASHBOARD_URL
    unset STORE_API_URL
    unset STORE_UPLOAD_URL
    unset UBUNTU_ONE_SSO_URL
    unset TEST_STORE
    export PS1="$ORIGINAL_PS1"
    unset ORIGINAL_PS1
    unset deactivate
}

export STORE_DASHBOARD_URL="https://dashboard.staging.snapcraft.io/"
export STORE_API_URL="https://api.staging.snapcraft.io/"
export STORE_UPLOAD_URL="https://storage.staging.snapcraftcontent.com/"
export UBUNTU_ONE_SSO_URL="https://login.staging.ubuntu.com/"
export TEST_STORE="staging"

export ORIGINAL_PS1="$PS1"
export PS1="$PS1 snapcraft staging servers> "
