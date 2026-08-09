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

"""The domain a test reports must never quietly be domain 0.

Domain 0 is the one value the whole domain scheme exists to keep tests off, and
a fallback to it cannot be caught anywhere downstream: a test that fell back is
still behind the wrapper and still carries MEDKIT_TEST_DOMAINS, so the
registration gate sees nothing wrong while the test shares a domain with every
process on the machine.
"""

import pytest

from ros2_medkit_test_utils.constants import get_test_domain_id


def test_an_unset_domain_is_refused(monkeypatch):
    monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
    with pytest.raises(RuntimeError) as excinfo:
        get_test_domain_id(0)
    assert 'not set' in str(excinfo.value)


def test_domain_zero_is_refused(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    with pytest.raises(RuntimeError) as excinfo:
        get_test_domain_id(0)
    assert 'sees every node' in str(excinfo.value)


def test_an_allocated_domain_is_returned(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '47')
    assert get_test_domain_id(0) == 47


def test_the_legacy_attribute_refuses_domain_zero_too(monkeypatch):
    from ros2_medkit_test_utils import constants
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    with pytest.raises(RuntimeError):
        constants.DEFAULT_DOMAIN_ID
    monkeypatch.setenv('ROS_DOMAIN_ID', '9')
    assert constants.DEFAULT_DOMAIN_ID == 9
