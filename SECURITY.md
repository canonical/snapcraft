# Security policy

## Release cycle

<!---
The information under this header may not be strictly accurate for all applications.
Review the wording carefully and only copy it if the support offered makes sense. If it
seems wrong, speak with Canonical Security Engineering about refining a version for
your application.
-->

Only the latest minor version of major versions supported in an Ubuntu LTS release
receive security support. Backports to previous minor releases or other unsupported
releases are made on a best-effort basis and will typically only be done for critical
vulnerabilities.

This support tracks the Ubuntu LTS release cycle. The most recent major version will
always support Ubuntu LTS series still in their regular maintenance lifecycle. When a
major Starcraft release drops support for an Ubuntu LTS series, the previous major
release remains supported for only the dropped LTS series until the end of its extended
support lifecycle.

Once an LTS release of Ubuntu is sunset, new security fixes for this software will no
longer be provided.

## Reporting a vulnerability

<!---
Replace the first link in this section with your repository's advisories board. See
GitHub's documentation for enabling the security advisory tab on a repository:
https://docs.github.com/en/code-security/security-advisories/working-with-repository-security-advisories/configuring-private-vulnerability-reporting-for-a-repository
-->

To report a security issue, file a [Private Security
Report](https://github.com/canonical/starcraft/security/advisories/new) with a
description of the issue, the steps you took to create the issue, affected versions,
and, if known, mitigations for the issue.

The [Ubuntu Security disclosure and embargo
policy](https://ubuntu.com/security/disclosure-policy) contains more information about
what you can expect when you contact us and what we expect from you.
