# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from snapcraft import formatting_utils


class DirtyReport:
    def __init__(self, *, dirty_properties=None, dirty_project_options=None,
                 changed_dependencies=None):
        self.dirty_properties = dirty_properties
        self.dirty_project_options = dirty_project_options
        self.changed_dependencies = changed_dependencies

    def report(self):
        messages = []

        if self.dirty_properties:
            humanized_properties = formatting_utils.humanize_list(
                self.dirty_properties, 'and')
            pluralized_connection = formatting_utils.pluralize(
                self.dirty_properties, 'property appears',
                'properties appear')
            messages.append(
                'The {} part {} to have changed.\n'.format(
                    humanized_properties, pluralized_connection))

        if self.dirty_project_options:
            humanized_options = formatting_utils.humanize_list(
                self.dirty_project_options, 'and')
            pluralized_connection = formatting_utils.pluralize(
                self.dirty_project_options, 'option appears',
                'options appear')
            messages.append(
                'The {} project {} to have changed.\n'.format(
                    humanized_options, pluralized_connection))

        if self.changed_dependencies:
            dependencies = [
                d['name'] for d in self.changed_dependencies]
            messages.append('{} changed: {}\n'.format(
                formatting_utils.pluralize(
                    dependencies, 'A dependency has',
                    'Some dependencies have'),
                formatting_utils.humanize_list(dependencies, 'and')))

        return ''.join(messages)

    def summary(self):
        reasons = []

        reason_count = 0
        if self.dirty_properties:
            reason_count += 1
        if self.dirty_project_options:
            reason_count += 1
        if self.changed_dependencies:
            reason_count += 1

        if self.dirty_properties:
            # Be specific only if this is the only reason
            if reason_count > 1 or len(self.dirty_properties) > 1:
                reasons.append('properties')
            else:
                reasons.append('{!r} property'.format(
                    next(iter(self.dirty_properties))))

        if self.dirty_project_options:
            # Be specific only if this is the only reason
            if reason_count > 1 or len(self.dirty_project_options) > 1:
                reasons.append('options')
            else:
                reasons.append('{!r} option'.format(
                    next(iter(self.dirty_project_options))))

        if self.changed_dependencies:
            # Be specific only if this is the only reason
            if reason_count > 1 or len(self.changed_dependencies) > 1:
                reasons.append('dependencies')
            else:
                reasons.append('{!r}'.format(
                    next(iter(self.changed_dependencies))['name']))

        return '{} changed'.format(
            formatting_utils.humanize_list(reasons, 'and', '{}'))
