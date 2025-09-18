## Installation

Snapcraft is available on all major Linux distributions, Windows, and macOS.

### Quick Start (Ubuntu & Snap-Enabled Distros)

**Install Snapcraft (recommended for most users):**

sudo snap install snapcraft --classic


**If snapd is missing on Ubuntu/Debian systems:**

sudo apt update && sudo apt install -y snapd


**For other distributions:**  
- On Fedora:

sudo dnf install snapd

- For all platforms, see the [official install guide](https://documentation.ubuntu.com/snapcraft/stable/how-to/setup/set-up-snapcraft).

### First Snap Build Example

Once Snapcraft is installed, you can build your first snap:

snapcraft init

_Edit the generated snapcraft.yaml file with your project details, then:_

snapcraft pack


**Documentation:**  
For complete tutorials and project examples, see the [Snapcraft documentation](https://documentation.ubuntu.com/snapcraft/stable).

