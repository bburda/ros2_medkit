# Copyright 2026 bburda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The manifest claim that black-box capture works on a stock install.

The plugin that provides a rosbag storage format lives in its own package.
`rosbag2_storage` is only the plugin interface and `libsqlite3-dev` is the C
library the fault database uses, so a manifest naming those two and stopping
leaves every storage plugin out of the install closure. What the user gets then
is not an error: RosbagCapture logs a warning and disables itself, so the feature
the README leads with is silently off.

Observed on the ROS build farm, whose chroot carries exactly `rosbag2_storage`
and `rosbag2_storage_mcap`, where `test_rosbag_capture` fails on
`Could not load/open plugin with storage id 'sqlite3'`.

Both plugins are load-bearing. mcap is the default the bags are written in.
An unknown format string still normalizes to sqlite3, but an unavailable
plugin falls back to whichever of the two is *not* configured - neither
backend is privileged over the other - and capture disables itself only
when neither one loads.
"""

import pathlib
import sys
import xml.etree.ElementTree as ElementTree

import pytest

MANIFEST = pathlib.Path(__file__).resolve().parent.parent / 'package.xml'

# The default backend. One name on every distro we release for.
MCAP_PLUGIN = 'rosbag2_storage_mcap'
# The sqlite3 backend, and what an unknown format string normalizes to. An
# *unavailable* configured format falls back to whichever plugin is not
# configured, not to this one specifically. On humble this name is the
# sqlite3 plugin itself; on jazzy and lyrical it is a metapackage that pulls
# sqlite3 together with mcap.
SQLITE3_PLUGIN = 'rosbag2_storage_default_plugins'


def declared(*tags):
    """Return the dependency names the manifest declares under the given tags."""
    root = ElementTree.parse(MANIFEST).getroot()
    names = set()
    for tag in tags:
        names.update(element.text.strip() for element in root.findall(tag))
    return names


def declared_elements(*tags):
    """Return the manifest elements under the given tags, not just their text."""
    root = ElementTree.parse(MANIFEST).getroot()
    elements = []
    for tag in tags:
        elements.extend(root.findall(tag))
    return elements


@pytest.mark.parametrize('plugin', [MCAP_PLUGIN, SQLITE3_PLUGIN])
def test_storage_plugin_is_an_exec_dependency(plugin):
    """An installed fault manager can open the formats it writes and falls back to."""
    names = declared('exec_depend', 'depend')
    assert plugin in names, (
        f'{plugin} is not an exec dependency, so a package install pulls in no such '
        f'rosbag storage plugin and black-box capture degrades or disables itself. '
        f'Declared: {sorted(names)}'
    )


@pytest.mark.parametrize('plugin', [MCAP_PLUGIN, SQLITE3_PLUGIN])
def test_storage_plugin_is_a_test_dependency(plugin):
    """
    The test chroot gets the plugins too, so the rosbag tests mean something.

    An exec dependency alone is not installed at build time, and build time is
    when the ROS build farm runs the tests.
    """
    names = declared('test_depend', 'depend')
    assert plugin in names, (
        f'{plugin} is not a test dependency, so the tests that assert rosbag capture '
        f'is enabled run without the plugin they need. Declared: {sorted(names)}'
    )


@pytest.mark.parametrize('plugin', [MCAP_PLUGIN, SQLITE3_PLUGIN])
def test_storage_plugin_dependency_has_no_condition(plugin):
    """
    A conditional dependency can pass every assertion above and still ship broken.

    ``<exec_depend condition="$ROS_DISTRO != 'humble'">rosbag2_storage_mcap</exec_depend>``
    still reads back as declared by ``declared()`` above - the tag and its text
    are untouched - but rosdep resolves nothing for it on the excluded distro,
    so the plugin is silently absent there exactly the way the missing
    declaration was before this task, just scoped to one distro instead of all
    of them. Covers both the ``exec_depend`` and ``test_depend`` declaration for
    each plugin, four in total.
    """
    offenders = [
        ElementTree.tostring(element, encoding='unicode').strip()
        for element in declared_elements('exec_depend', 'test_depend', 'depend')
        if element.text and element.text.strip() == plugin and element.get('condition') is not None
    ]
    assert not offenders, (
        f'{plugin} is declared with a condition attribute, which can silently drop it '
        f'from the install closure on the distro(s) the condition excludes: {offenders}'
    )


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
