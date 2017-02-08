#!/usr/bin/env fish

function deactivate
    set -e UBUNTU_STORE_API_ROOT_URL
    set -e UBUNTU_STORE_SEARCH_ROOT_URL
    set -e UBUNTU_STORE_UPLOAD_ROOT_URL
    set -e UBUNTU_SSO_API_ROOT_URL
    set -e TEST_STORE

    set -g -x PS1 "$ORIGINAL_PS1"
    set -e ORIGINAL_PS1
    set -e deactivate
end

set -g -x UBUNTU_STORE_API_ROOT_URL "https://myapps.developer.staging.ubuntu.com/dev/api/"
set -g -x UBUNTU_STORE_SEARCH_ROOT_URL "https://search.apps.staging.ubuntu.com/"
set -g -x UBUNTU_STORE_UPLOAD_ROOT_URL "https://upload.apps.staging.ubuntu.com/"
set -g -x UBUNTU_SSO_API_ROOT_URL "https://login.staging.ubuntu.com/api/v2/"
set -g -x TEST_STORE "staging"

set -g -x ORIGINAL_PS1 "$PS1"
set -g -x PS1 "$PS1 snapcraft staging servers> :"
