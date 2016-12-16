# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


def combine_paths(paths, prepend, separator):
    """Combine list of paths into a string.

    :param list paths: List of paths to stringify.
    :param str prepend: String to prepend to each path in the string.
    :param str separator: String to place between each path in the string.
    """

    paths = ['{}{}'.format(prepend, p) for p in paths]
    return separator.join(paths)


def format_path_variable(envvar, paths, prepend, separator):
    """Return a path-like environment variable definition that appends.

    :param str envvar: The environment variable in question.
    :param list paths: The paths to append to the environment variable.
    :param str prepend: String to prepend to each path in the definition.
    :param str separator: String to place between each path in the definition.
    """

    if not paths:
        raise ValueError(
            "Failed to format '${}': no paths supplied".format(envvar))

    return '{envvar}="${envvar}{separator}{paths}"'.format(
        envvar=envvar, separator=separator, paths=combine_paths(
            paths, prepend, separator))


def humanize_list(items, conjunction):
    """Format a list into a human-readable string.

    :param list items: List to humanize.
    :param str conjunction: The conjunction used to join the final element to
                            the rest of the list (e.g. 'and').
    """

    if len(items) == 0:
        return ''

    quoted_items = ['{!r}'.format(item) for item in sorted(items)]
    if len(items) == 1:
        return quoted_items[0]

    humanized = ', '.join(quoted_items[:-1])

    if len(items) > 2:
        humanized += ','

    return '{} {} {}'.format(humanized, conjunction, quoted_items[-1])


def pluralize(container, if_one, if_multiple):
    if len(container) == 1:
        return if_one
    else:
        return if_multiple
