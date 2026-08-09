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

"""The safe band and the allocator that draws from it.

These are the properties that do not need a process to prove: which domains the
band contains, that the selector never leaves it, that a lock port already held
by somebody else is stepped over rather than fatal, that a full band is reported
as such instead of quietly becoming domain 0, that the band is checked against
the kernel range it is derived from, and that a caller waiting for several
domains holds none of them meanwhile.
"""

import contextlib
import errno
import itertools
import socket
import threading
import time

import medkit_domain
import pytest

EXPECTED_BAND = tuple(range(1, 101)) + tuple(range(215, 232))


class ReservedBand:
    """A handful of domains this process has proved it can hold, and holds.

    The lock port of a domain is ``32768 + domain``, which is the floor of the
    kernel's default ephemeral range: any process on the machine can be handed
    one of those ports at any moment. That is the whole reason the allocator
    skips a domain whose port it cannot bind, and a test helper that assumed it
    could bind all 117 at once was making exactly the assumption the band
    reasoning rejects. So a test picks its band by binding, not by naming: what
    it gets is whatever it actually holds.

    ``release(n)`` hands domains back so the allocator can take them, which is
    how a test arranges a known number of free slots.
    """

    def __init__(self, domains, sockets):
        self.domains = list(domains)
        self._sockets = list(sockets)

    @property
    def spec(self):
        """Format the band the way MEDKIT_TEST_DOMAIN_BAND wants it."""
        return ','.join(str(domain) for domain in self.domains)

    def release(self, count=None):
        """Give back *count* domains, or all of them, and return which."""
        count = len(self._sockets) if count is None else count
        freed = []
        for _ in range(count):
            if not self._sockets:
                break
            sock = self._sockets.pop()
            freed.append(self.domains[len(self._sockets)])
            sock.close()
        return freed

    def close(self):
        for sock in self._sockets:
            sock.close()
        self._sockets = []


@contextlib.contextmanager
def reserve_band(count, candidates=None):
    """Hold *count* domains chosen because their lock port could be bound.

    Never assumes a particular domain is free. Fails only if the machine cannot
    spare *count* of the whole band, which is a broken machine rather than a
    test that got unlucky.
    """
    candidates = medkit_domain.USABLE_DOMAINS if candidates is None else candidates
    domains, sockets = [], []
    for domain in candidates:
        if len(domains) == count:
            break
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.bind(('', medkit_domain.PORT_BASE + domain))
        except OSError:
            sock.close()
            continue
        domains.append(domain)
        sockets.append(sock)
    reserved = ReservedBand(domains, sockets)
    try:
        assert len(domains) == count, (
            f'could only hold {len(domains)} of {count} domains out of '
            f'{len(candidates)} candidates; this machine has almost no free lock ports'
        )
        yield reserved
    finally:
        reserved.close()


def test_band_is_the_documented_range():
    assert medkit_domain.USABLE_DOMAINS == EXPECTED_BAND
    assert len(medkit_domain.USABLE_DOMAINS) == 117


def test_band_endpoints_are_included():
    for endpoint in (1, 100, 215, 231):
        assert endpoint in medkit_domain.USABLE_DOMAINS


def test_band_excludes_the_domains_that_are_not_safe():
    # 0 is the developer default, 101-214 overlap the kernel ephemeral range,
    # 232 runs past the top of the UDP port space.
    for unsafe in (0, 101, 150, 214, 232, 233):
        assert unsafe not in medkit_domain.USABLE_DOMAINS


def test_selector_stays_inside_the_band_and_covers_all_of_it():
    selector = medkit_domain.BandSelector(medkit_domain.USABLE_DOMAINS, start=0)
    drawn = [selector() for _ in range(len(EXPECTED_BAND) * 2)]
    assert set(drawn) <= set(EXPECTED_BAND)
    assert set(drawn) == set(EXPECTED_BAND)


def test_selector_starts_somewhere_other_than_the_first_domain_sometimes():
    starts = set()
    for _ in range(50):
        starts.add(medkit_domain.BandSelector(medkit_domain.USABLE_DOMAINS)())
    assert len(starts) > 1, 'every allocator starting at the same domain maximises contention'


@pytest.mark.parametrize('endpoint', [1, 100, 215, 231])
def test_a_band_of_one_endpoint_yields_that_endpoint_or_refuses(endpoint):
    # Whether this endpoint is free right now is not ours to decide: its lock
    # port sits in the range the kernel hands out. Both outcomes are asserted,
    # so the test never assumes and never skips. What it pins is that the
    # selector cannot leave the band it was given.
    try:
        with medkit_domain.hold_domains(1, domains=(endpoint,), wait_timeout=0.5) as domains:
            assert domains == [endpoint]
    except medkit_domain.NoDomainAvailable:
        pass


def test_a_lock_port_held_by_an_outsider_is_skipped_not_fatal():
    with reserve_band(3) as band:
        blocked = band.domains[0]
        band.release(2)  # the other two go back; only `blocked` stays held
        with medkit_domain.hold_domains(2, domains=band.domains) as domains:
            assert blocked not in domains
            assert sorted(domains) == sorted(band.domains[1:])


def test_a_band_with_every_port_held_is_reported_not_silently_downgraded():
    reached_body = False
    with reserve_band(3) as band:
        with pytest.raises(medkit_domain.NoDomainAvailable) as excinfo:
            with medkit_domain.hold_domains(1, domains=band.domains, wait_timeout=0.5):
                reached_body = True
        message = str(excinfo.value)
    assert not reached_body, 'a full band must not hand out a domain'
    assert 'ROS_DOMAIN_ID' in message
    assert '3' in message, 'the message must say how large the band it exhausted is'


def test_concurrent_holders_never_share_a_domain():
    with reserve_band(2) as band:
        band.release()
        with medkit_domain.hold_domains(1, domains=band.domains) as first:
            with medkit_domain.hold_domains(1, domains=band.domains) as second:
                assert first[0] != second[0]


def test_domains_are_released_when_the_holder_exits():
    with reserve_band(1) as band:
        only = band.release()[0]
        with medkit_domain.hold_domains(1, domains=(only,)) as domains:
            assert domains == [only]
        with medkit_domain.hold_domains(1, domains=(only,)) as domains:
            assert domains == [only]


def test_a_failure_inside_the_body_still_releases_the_domain():
    # domain_coordinator.domain_id() swallows OSError around its own yield, so an
    # OSError escaping the body must not be able to confuse the release path.
    with reserve_band(1) as band:
        only = band.release()[0]
        with pytest.raises(OSError):
            with medkit_domain.hold_domains(1, domains=(only,)):
                raise OSError('the test under the allocator blew up')
        with medkit_domain.hold_domains(1, domains=(only,)) as domains:
            assert domains == [only]


def test_a_band_override_can_narrow_the_band_but_never_widen_it():
    assert medkit_domain.parse_band('1-4,215') == (1, 2, 3, 4, 215)
    assert medkit_domain.default_band({medkit_domain.BAND_ENV: '7,8'}) == (7, 8)
    assert medkit_domain.default_band({}) == medkit_domain.USABLE_DOMAINS
    for unsafe in ('0', '101', '232', '1-101'):
        with pytest.raises(ValueError) as excinfo:
            medkit_domain.parse_band(unsafe)
        assert 'safe band excludes' in str(excinfo.value)


def test_environment_export_names_the_primary_and_the_extras():
    env = {'ROS_DOMAIN_ID': '0', 'MEDKIT_SECONDARY_DOMAINS': '229'}
    medkit_domain.apply_to_environ([7, 8, 9], env)
    assert env['ROS_DOMAIN_ID'] == '7'
    assert env['MEDKIT_SECONDARY_DOMAINS'] == '8,9'

    medkit_domain.apply_to_environ([7], env)
    assert env['ROS_DOMAIN_ID'] == '7'
    assert 'MEDKIT_SECONDARY_DOMAINS' not in env


def test_a_refused_bind_is_not_reported_as_contention(monkeypatch):
    # domain_coordinator throws the errno away, so a machine that refuses the
    # bind outright looks exactly like a busy band from above. Reported as
    # contention it would advise lowering the test parallelism, which fixes
    # nothing and reads as this package being broken.
    import socket as socket_module

    real_socket = socket_module.socket

    class RefusingSocket(real_socket):

        def bind(self, address):
            raise OSError(errno.EACCES, 'Permission denied')

    monkeypatch.setattr(socket_module, 'socket', RefusingSocket)
    reason = medkit_domain.diagnose_bind_refusal(medkit_domain.USABLE_DOMAINS)
    assert reason is not None
    assert 'EACCES' in reason
    assert 'not a fault in the package under test' in reason


def test_a_busy_port_is_reported_as_contention_and_nothing_else():
    with reserve_band(2) as band:
        assert medkit_domain.diagnose_bind_refusal(band.domains) is None


def test_a_machine_that_refuses_binds_says_so_instead_of_blaming_parallelism(monkeypatch):
    import socket as socket_module

    real_socket = socket_module.socket

    class RefusingSocket(real_socket):

        def bind(self, address):
            raise OSError(errno.EACCES, 'Permission denied')

    with reserve_band(1) as band:
        band.release()
        monkeypatch.setattr(socket_module, 'socket', RefusingSocket)
        with pytest.raises(medkit_domain.LockPortsUnusable) as excinfo:
            with medkit_domain.hold_domains(1, domains=band.domains, wait_timeout=0.5):
                pass
    message = str(excinfo.value)
    assert 'lower the test parallelism' not in message
    assert 'seccomp' in message or 'EACCES' in message


def test_every_allocation_failure_shares_a_base_the_wrappers_catch():
    for failure in (
        medkit_domain.NoDomainAvailable,
        medkit_domain.LockPortsUnusable,
        medkit_domain.EphemeralRangeUnknown,
        medkit_domain.UnsafeBand,
    ):
        assert issubclass(failure, medkit_domain.DomainAllocationError)


# ---------------------------------------------------------------------------
# The band against the kernel range it is derived from
# ---------------------------------------------------------------------------
#
# The band is a constant, and it is only correct for a machine whose
# net.ipv4.ip_local_port_range is the Linux default. That sysctl is routinely
# changed on CI images and Kubernetes nodes, and the failure it causes does not
# point at itself: a node dies with "failed to bind to ANY:<port>: address in
# use" in whichever test happened to be running.


DEFAULT_RANGE = (32768, 60999)

#: Widened all the way down. A common hardening and scaling setting, and it
#: leaves no domain in the band safe at all.
WIDE_RANGE = (1024, 65535)

#: What a Kubernetes node or a container image with many outbound connections
#: often carries. Part of the band survives it, which is the more interesting
#: case: a check that only caught "everything is broken" would miss it.
CI_RANGE = (10240, 65535)


def declare(low, high):
    """Build the environment of a machine that declares its range as low-high."""
    return {medkit_domain.EPHEMERAL_RANGE_ENV: f'{low}-{high}'}


def test_this_machine_reports_a_range_and_the_band_holds_on_it():
    low, high, source = medkit_domain.read_ephemeral_range()
    assert 0 < low <= high <= medkit_domain.MAX_UDP_PORT
    assert medkit_domain.EPHEMERAL_RANGE_FILE in source
    # Not "the default": whatever this machine says. If the band did not hold
    # here, every test in the workspace would be running on a domain the kernel
    # may also hand to something else, and that is worth failing over.
    assert medkit_domain.verify_band(medkit_domain.USABLE_DOMAINS)[:2] == (low, high)


@pytest.mark.parametrize(
    'domain,safe',
    [
        (1, True), (100, True), (215, True), (231, True),   # the band's endpoints
        (101, False),                                       # 32650-32899, first overlap
        (214, False),                                       # 60900-61149, last overlap
        (232, False),                                       # 65400-65649, past the ceiling
        (233, False),
    ],
)
def test_the_band_endpoints_are_exactly_the_safe_domains_under_the_default_range(domain, safe):
    assert medkit_domain.domain_is_safe(domain, *DEFAULT_RANGE) is safe
    assert (domain in medkit_domain.USABLE_DOMAINS) is safe


def test_the_whole_band_is_safe_under_the_default_range_and_nothing_else_is():
    low, high = DEFAULT_RANGE
    assert medkit_domain.unsafe_domains(medkit_domain.USABLE_DOMAINS, low, high) == ()
    outside = [d for d in range(0, 240) if d not in medkit_domain.USABLE_DOMAINS and d]
    assert set(medkit_domain.unsafe_domains(outside, low, high)) == set(outside)


def test_a_range_that_swallows_the_band_is_refused_and_names_the_sysctl():
    with pytest.raises(medkit_domain.UnsafeBand) as excinfo:
        medkit_domain.verify_band(medkit_domain.USABLE_DOMAINS, declare(*WIDE_RANGE))
    message = str(excinfo.value)
    assert 'net.ipv4.ip_local_port_range' in message
    assert '1024-65535' in message
    # No subset of the band survives this one, and the message has to say so
    # rather than offer a band override that cannot help.
    assert 'no domain in the band is safe' in message


def test_a_range_that_leaves_part_of_the_band_offers_that_part_explicitly():
    with pytest.raises(medkit_domain.UnsafeBand) as excinfo:
        medkit_domain.verify_band(medkit_domain.USABLE_DOMAINS, declare(*CI_RANGE))
    message = str(excinfo.value)
    # 7400 + 250 * 10 + 249 = 10149, the last slice that ends below 10240.
    assert f'{medkit_domain.BAND_ENV}=1-10' in message
    assert '10 of 117 domains' in message
    assert 'net.ipv4.ip_local_port_range' in message


def test_a_narrowed_band_that_is_safe_on_this_machine_is_accepted():
    # The documented way out: narrow the band to the part that still holds. It
    # has to be accepted, or the advice in the failure message is a dead end.
    safe_here = medkit_domain.parse_band('1-10')
    assert medkit_domain.verify_band(safe_here, declare(*CI_RANGE))[:2] == CI_RANGE


def test_holding_a_domain_is_refused_outright_on_a_machine_the_band_does_not_fit(monkeypatch):
    monkeypatch.setenv(medkit_domain.EPHEMERAL_RANGE_ENV, '1024-65535')
    reached_body = False
    with pytest.raises(medkit_domain.UnsafeBand):
        with medkit_domain.hold_domains(1, wait_timeout=0.5):
            reached_body = True
    assert not reached_body, 'a domain was handed out on a machine where none is safe'


def test_a_declaration_can_only_widen_the_kernel_range_never_narrow_it():
    # Otherwise the declaration is a way to silence the check, and a check that
    # can be silenced by an environment variable is not a check.
    low, high, _ = medkit_domain.read_ephemeral_range(declare(40000, 40010))
    kernel_low, kernel_high, _ = medkit_domain.read_ephemeral_range({})
    assert low == min(40000, kernel_low)
    assert high == max(40010, kernel_high)


def test_a_machine_that_will_not_say_is_refused_rather_than_assumed_to_be_default(tmp_path):
    with pytest.raises(medkit_domain.EphemeralRangeUnknown) as excinfo:
        medkit_domain.read_ephemeral_range({}, str(tmp_path / 'no_such_file'))
    message = str(excinfo.value)
    assert medkit_domain.EPHEMERAL_RANGE_ENV in message
    assert '32768' not in message, 'the default must not be presented as an answer'


def test_a_declaration_carries_a_machine_that_cannot_read_the_file(tmp_path):
    # /proc/sys is readable inside a container and unwritable there, so this is
    # for the case where it is not visible at all.
    low, high, source = medkit_domain.read_ephemeral_range(
        declare(*DEFAULT_RANGE), str(tmp_path / 'no_such_file')
    )
    assert (low, high) == DEFAULT_RANGE
    assert medkit_domain.EPHEMERAL_RANGE_ENV in source
    assert medkit_domain.verify_band(medkit_domain.USABLE_DOMAINS, declare(*DEFAULT_RANGE))


@pytest.mark.parametrize('spec', ['', 'nonsense', '32768', '1024-70000', '60999-32768', '0-99'])
def test_a_malformed_range_declaration_is_rejected_not_ignored(spec):
    with pytest.raises((ValueError, medkit_domain.EphemeralRangeUnknown)):
        medkit_domain.read_ephemeral_range(
            {medkit_domain.EPHEMERAL_RANGE_ENV: spec}, '/nonexistent/ip_local_port_range'
        )


def test_the_band_renders_the_way_the_override_reads_it():
    assert medkit_domain.format_band((1, 2, 3, 5, 215)) == '1-3,5,215'
    assert medkit_domain.format_band(()) == '<none>'
    assert medkit_domain.parse_band(
        medkit_domain.format_band(medkit_domain.USABLE_DOMAINS)
    ) == medkit_domain.USABLE_DOMAINS


# ---------------------------------------------------------------------------
# Several domains at once, without holding what somebody else is waiting for
# ---------------------------------------------------------------------------


def bind_all(domains):
    """Bind every lock port in *domains* at once, or nothing. Caller closes."""
    sockets = []
    for domain in domains:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.bind(('', medkit_domain.PORT_BASE + domain))
        except OSError:
            sock.close()
            for held in sockets:
                held.close()
            return None
        sockets.append(sock)
    return sockets


def wait_for_bind(domains, timeout):
    """Wait until every lock port in *domains* can be held at the same moment."""
    deadline = time.monotonic() + timeout
    while True:
        sockets = bind_all(domains)
        if sockets is not None:
            return sockets
        if time.monotonic() >= deadline:
            return None
        time.sleep(0.05)


def test_a_caller_that_cannot_complete_its_set_holds_nothing_while_it_waits():
    # Three domains, one of them kept by this test, and a caller that asks for
    # all three. It can never succeed, so what it does while failing is the
    # whole question: if it keeps the two it can get, those two are gone for
    # everybody else for as long as it waits, and a second caller of the same
    # shape wedges against it until both wait budgets run out.
    with reserve_band(3) as band:
        free = sorted(band.release(2))
        waiting = threading.Event()
        outcome = {}

        def caller():
            try:
                with medkit_domain.hold_domains(
                    3,
                    domains=band.domains,
                    wait_timeout=10,
                    announce=lambda message: waiting.set(),
                ):
                    outcome['held'] = True
            except BaseException as error:  # noqa: B902 - recorded, not swallowed
                outcome['refused'] = error

        thread = threading.Thread(target=caller, daemon=True)
        thread.start()
        sockets = None
        try:
            assert waiting.wait(30), 'the caller never announced that it was waiting'
            sockets = wait_for_bind(free, 5)
            assert sockets is not None, (
                'the caller kept part of its set while waiting for the rest, so the '
                f'domains it could not use ({free}) were unavailable to anybody else'
            )
        finally:
            for sock in sockets or []:
                sock.close()
            thread.join(60)
    assert 'held' not in outcome, 'a set that cannot be completed must not be handed out'
    assert isinstance(outcome.get('refused'), medkit_domain.NoDomainAvailable)


def test_two_concurrent_multi_domain_callers_both_get_their_set(monkeypatch):
    # The case that used to wedge: two callers wanting three domains each from a
    # band of four. Left to chance it happens about one run in six, so the
    # interleaving is forced instead of waited for - the first two takes of each
    # caller alternate, which puts two domains in each caller's hands and none
    # in the band. From there, incremental acquisition has nothing to give
    # either of them and never will.
    #
    # Only the first four takes are gated. After that the handover is out of the
    # way and the allocator is on its own.
    with reserve_band(4) as band:
        band.release()
        pool = tuple(band.domains)

        handover = threading.Barrier(2, timeout=60)
        takes = itertools.count()
        counter_lock = threading.Lock()
        real_domain_id = medkit_domain.domain_coordinator.domain_id

        def gate():
            with counter_lock:
                index = next(takes)
            if index < 4:
                try:
                    handover.wait()
                except threading.BrokenBarrierError:
                    pass  # degrade to no interleaving rather than to an error

        class Handover:

            def __init__(self, inner):
                self._inner = inner

            def __enter__(self):
                domain = self._inner.__enter__()
                gate()
                return domain

            def __exit__(self, *exc_info):
                return self._inner.__exit__(*exc_info)

        monkeypatch.setattr(
            medkit_domain.domain_coordinator,
            'domain_id',
            lambda selector: Handover(real_domain_id(selector)),
        )

        results = {}

        def caller(name):
            try:
                with medkit_domain.hold_domains(3, domains=pool, wait_timeout=30) as held:
                    results[name] = sorted(held)
                    # Hold long enough that the other caller really has to wait
                    # for this one rather than slipping past it.
                    time.sleep(0.2)
            except BaseException as error:  # noqa: B902 - recorded, not swallowed
                results[name] = error

        threads = [
            threading.Thread(target=caller, args=(name,), daemon=True)
            for name in ('first', 'second')
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(120)
        for thread in threads:
            assert not thread.is_alive(), 'a caller never finished'

    for name in ('first', 'second'):
        got = results.get(name)
        assert not isinstance(got, BaseException), f'{name} caller failed: {got!r}'
        assert got is not None, f'{name} caller recorded nothing'
        assert len(got) == 3 and len(set(got)) == 3, f'{name} got {got}'
        assert set(got) <= set(pool), f'{name} left the band it was given: {got}'
