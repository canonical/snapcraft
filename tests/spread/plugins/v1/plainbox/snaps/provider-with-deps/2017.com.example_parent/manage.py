#!/usr/bin/env python3
from plainbox.provider_manager import setup, N_


setup(
    name="plainbox-provider-parent",
    namespace="2017.com.example.parent",
    version="1.0",
    description=N_("A parent Plainbox provider"),
    gettext_domain="2017_com_example_parent",
)
