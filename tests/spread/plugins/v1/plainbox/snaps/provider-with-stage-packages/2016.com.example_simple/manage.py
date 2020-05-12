#!/usr/bin/env python3
from plainbox.provider_manager import setup, N_


setup(
    name="plainbox-provider-simple",
    namespace="2016.com.example",
    version="1.0",
    description=N_("A really simple Plainbox provider"),
    gettext_domain="2016_com_example_simple",
)
