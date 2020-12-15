#!/usr/bin/env python3
from plainbox.provider_manager import N_, setup

setup(
    name="plainbox-provider-child",
    namespace="2017.com.example.child",
    version="1.0",
    description=N_("A child Plainbox provider"),
    gettext_domain="2017_com_example_child",
)
