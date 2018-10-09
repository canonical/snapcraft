#!/bin/sh -e

export CONFIG_FILE_PATH="$HOME/.config/snapcraft/cli.cfg"
mkdir -p "$(dirname "$CONFIG_FILE_PATH")"

set_outdated_step_action()
{
	cat <<- EOF > "$CONFIG_FILE_PATH"
		[Lifecycle]
		outdated_step_action = $1
		EOF
}

clear_config()
{
	rm -f "$CONFIG_FILE_PATH"
}
