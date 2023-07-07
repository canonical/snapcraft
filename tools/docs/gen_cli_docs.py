#!/usr/bin/env python3

import argparse
from craft_cli.dispatcher import _CustomArgumentParser, Dispatcher
import os
import pathlib
import sys

this_dir = pathlib.Path(os.path.split(__file__)[0])
sys.path.insert(0, str((this_dir / ".." / "..").absolute()))

from snapcraft import cli


def command_page_header(cmd, options_str, required_str):
    underline = "=" * len(cmd.name)
    return f"""
.. _ref_commands_{cmd.name}:

{cmd.name}
{underline}
{cmd.overview}

Usage
-----

:command:`snapcraft {cmd.name}{options_str}{required_str}`

"""


def argname(dest, metavars):
    # There should be only one metavar name. If None, use the dest instead.
    if metavars[0]:
        return metavars[0]
    else:
        return dest


def make_sentence(t):
    if not t:
        t = ""
    return t[:1].upper() + t[1:].rstrip(".") + "."


def not_none(*args):
    return [x for x in args if x != None]


def make_section(title, items):
    s = ""
    if items:
        underline = "-" * len(title)
        s = f"{title}\n{underline}\n\n"

    for dest, (names, help_str) in items:
        if help_str != "==SUPPRESS==":
            if names == [None]:
                s += f"``{dest}``\n"
            else:
                s += " or ".join([f"``{name}``" for name in names]) + "\n"
            if help_str:
                s += "   %s\n" % make_sentence(help_str)

    s += "\n"
    return s


def remove_spaces(t):
    return t.replace(" ", "-")


def main(docs_dir):
    """Generate reference documentation for the command line interface,
    creating pages in the docs/reference/commands directory and creating the
    directory itself if necessary."""

    # Create the directory for the commands reference.
    commands_ref_dir = docs_dir / "reference" / "commands"
    if not commands_ref_dir.exists():
        commands_ref_dir.mkdir()

    # Create a dispatcher like Snapcraft does to get access to the same options.
    dispatcher = cli.get_dispatcher()

    help_builder = dispatcher._help_builder

    global_options = {}
    for arg in dispatcher.global_arguments:
        opts = not_none(arg.short_option, arg.long_option)
        global_options[arg.name] = (opts, arg.help_message)

    toc = []

    for group in cli.COMMAND_GROUPS:
        group_name = remove_spaces(group.name.lower()) + "-commands" + os.extsep + "rst"
        group_path = commands_ref_dir / group_name
        g = group_path.open("w")

        for cmd_class in sorted(group.commands, key=lambda c: c.name):
            cmd = cmd_class({})
            p = _CustomArgumentParser(help_builder)
            cmd.fill_parser(p)

            options = {}
            required = []
            options_str = " "

            for action in p._actions:
                if action.option_strings and action.dest not in global_options:
                    options[action.dest] = (action.option_strings, action.help)
                elif action.required or not action.option_strings:
                    required.append((action.dest, ([action.metavar], action.help)))

            cmd_path = commands_ref_dir / (cmd.name + os.extsep + "rst")

            if options or global_options:
                options_str += "[options]"
            required_str = "".join(
                [(" <%s>" % argname(d, vl)) for d, (vl, h) in required]
            )

            f = cmd_path.open("w")
            f.write(command_page_header(cmd, options_str, required_str))
            f.write(make_section("Required", required))
            f.write(make_section("Options", sorted(options.items())))
            f.write(make_section("Global options", sorted(global_options.items())))

            # Add a section for the command to be included in the group reference.
            g.write(f":ref:`ref_commands_{cmd.name}`\n")
            g.write("   " + make_sentence(cmd.help_msg).replace("\n", "\n   ") + "\n\n")

            # Add an entry in the table of contents.
            toc.append(cmd.name)

    toc_path = commands_ref_dir / "toc.rst"
    f = toc_path.open("w")
    f.write(".. toctree::\n   :hidden:\n\n")
    for name in sorted(toc):
        f.write(f"   /reference/commands/{name}\n")


if __name__ == "__main__":
    if len(sys.argv) == 2:
        main(pathlib.Path(sys.argv[1]))
