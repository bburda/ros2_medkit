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

"""Comparing two served documents that may take a moment to agree.

Two gateways on one ROS graph can disagree for an instant while one of them is
mid-refresh, so equality needs a grace period. Getting that grace period wrong
in the obvious way produces a test that passes having compared nothing:

    left = right = None
    try:
        left, right = fetch_left(), fetch_right()   # tuple built THEN unpacked
    except RequestException:
        pass
    ...
    assertEqual(left, right)                        # None == None -> green

If either fetch raises, Python never unpacks, both names keep their initial
``None``, and the final assertion compares two ``None`` values and passes. A
gateway that was down for the entire window therefore looked like a match - and
in the test this was written for, the right-hand gateway is the thing under
test. This module exists so there is one implementation of the comparison and
it fails loudly instead.
"""

import time

DEFAULT_TIMEOUT_SEC = 15.0
DEFAULT_INTERVAL_SEC = 0.5


class DocumentsNeverFetched(AssertionError):
    """Raised when no successful pair of documents was ever obtained."""


def assert_documents_match(
    test,
    fetch_left,
    fetch_right,
    *,
    what,
    timeout=DEFAULT_TIMEOUT_SEC,
    interval=DEFAULT_INTERVAL_SEC,
):
    """Assert two documents converge, distinguishing "differ" from "never read".

    Retries until *timeout*. Three outcomes, all of them explicit:

    * a pair was fetched and matched -> returns ``(left, right)``
    * a pair was fetched and never matched -> ``assertEqual`` shows the diff
    * no pair was ever fetched -> fails naming the last error, rather than
      quietly comparing two placeholder values

    Both fetches are evaluated separately so a failure on either side is
    attributed, instead of being lost in an all-or-nothing tuple unpack.
    """
    deadline = time.monotonic() + timeout
    left = right = None
    fetched = False
    last_error = None

    while True:
        try:
            left = fetch_left()
            right = fetch_right()
            fetched = True
            if left == right:
                return left, right
        except Exception as exc:  # noqa: B902 - re-raised below if never fetched
            last_error = exc
        if time.monotonic() >= deadline:
            break
        time.sleep(interval)

    if not fetched:
        raise DocumentsNeverFetched(
            f'{what}: no successful pair of documents was fetched in '
            f'{timeout}s, so nothing was ever compared. Last error: '
            f'{last_error!r}'
        )

    test.assertEqual(left, right, f'{what} still differed after {timeout}s')
    return left, right
