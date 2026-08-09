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

"""Runtime ``ROS_DOMAIN_ID`` allocation for tests.

A test asks for one or more domains when it starts and holds them, through an
open socket, for exactly as long as it runs. The kernel closes that socket when
the holder dies for any reason, including SIGKILL, so a domain is never leaked
by a crashed run.

The lock is ``domain_coordinator.domain_id`` from ``ament_cmake_ros``, which is
installed on every distro we build for. It binds a TCP socket on
``32768 + domain`` and holds it for the lifetime of its context. That socket is
what carries the allocation past a single CTest run: two ``ctest`` processes
started by two different ``colcon`` package jobs contend for the same ports,
which no CTest ``RESOURCE_LOCK`` can do.

The reach of that lock is one NETWORK NAMESPACE, not one machine, and the
difference matters. Two containers on the same host each get their own
namespace, so each can bind ``32768 + 47`` and believe it holds domain 47 - yet
DDS multicast crosses Docker's default bridge, so nodes in those two containers
do see each other. Domain isolation between containers on a shared bridge is
therefore not something this scheme provides. Inherited from
``domain_coordinator`` and true of upstream's ``ament_add_ros_isolated_gtest``
in the same way, so it is a limit to know about rather than a bug to fix here:
isolate containers at the network, or give them separate ``ROS_DOMAIN_ID``
ranges, if they must not meet.

Three things here are ours rather than upstream's:

* the band, see ``USABLE_DOMAINS`` below, and the run-time check that the band
  is actually safe on the machine the tests run on, see ``verify_band``;
* waiting. ``domain_id`` raises as soon as it has tried its selector 100 times,
  which under ``ctest -j`` simply means "everything is busy right now". A test
  that cannot have a domain yet has to wait for one, with a bound, not fail.
* all-or-nothing acquisition for a test that needs several domains, see
  ``hold_domains``.

---------------------------------------------------------------------------
Why the band is not simply 1-232
---------------------------------------------------------------------------

RTPS gives every DDS domain a 250-port slice of the UDP space:

  slice(d) = [7400 + 250 * d, 7400 + 250 * d + 249]

Cyclone DDS 0.10.5, what both Humble and Jazzy ship, binds the multicast
sockets at offsets 0 and 1 unconditionally, on every configuration -
ddsi_portmapping.c forces the participant index to 0 for both multicast cases,
so the index never enters into it. Those sockets do set both SO_REUSEPORT and
SO_REUSEADDR, but a collision there is still fatal: Linux only grants the reuse
exemption when every socket that ever held the port opted in, so an ordinary
socket already sitting on it blocks the bind regardless. A failed multicast bind
aborts domain creation, so this is the failure that actually kills a node:

  ddsi_udp_create_conn: failed to bind to ANY:53150: address in use
  rmw_create_node: failed to create domain, error Error

53150 is 7400 + 250 * 183, the multicast base port of domain 183 - the failure
above is exactly the case this band protects against.

The unicast sockets, at offsets 10 + 2p and 11 + 2p for participant index p,
depend on Discovery/ParticipantIndex. On Humble, rmw_cyclonedds_cpp emits no
<Discovery> element, so the default "none" applies and the single unicast socket
gets a kernel-assigned port with no relation to the domain. On Jazzy,
rmw_cyclonedds_cpp injects <ParticipantIndex>auto</ParticipantIndex> with
MaxAutoParticipantIndex 32, so unicast does land on offsets 10 through 75 - but
a collision there is not fatal, it just advances to the next participant index.

Fast DDS 3.6.2 (Lyrical) derives every listening port from the formula and never
falls back to a kernel-assigned one; its participant id mutates by +2 for up to
100 tries on collision. A failed multicast bind there is only a warning, so
discovery degrades instead of the process dying.

What keeping a domain's slice out of the ephemeral range buys, then: on every
distro it protects offsets 0 and 1, the one collision that is fatal. On Jazzy it
additionally covers offsets 10 through 75. On Humble the unicast socket sits
outside the scheme entirely and needs no protection, since a kernel-assigned
port is free by construction.

The protection is deterministic rather than statistical: once a socket with the
reuse options holds a port, the kernel will not also hand that port out as an
ephemeral one. The race runs one way - the unrelated process has to grab the
port before the participant starts, never the other way around.

The kernel hands out ephemeral ports from net.ipv4.ip_local_port_range, which
defaults to 32768-60999. Any unrelated process - a browser, a package manager, a
second test run - can be given a port in that range, and if it is one of ours
the whole domain stops working. Mapped back through the formula, the default
range covers domains 101 to 214: domain 101's slice, 32650-32899, already
overlaps the range starting at 32768.

The usable domains are the ones whose whole slice sits outside that range: 1-100
and 215-231. Domain 0 stays free because it is the ROS 2 default a developer
shell uses, and 232 is dropped because its slice runs past 65535. Upstream's
``domain_coordinator`` selector uses 1-100 only; the extra 17 are ours and are
the reason a selector is passed in at all.

That derivation is only true for a machine whose range is the default one, and
the range is a sysctl any operator, image or Kubernetes node can change. With
``net.ipv4.ip_local_port_range`` set to ``1024 65535``, which is a common
hardening and scaling setting, every domain in the band above is inside it and
45% of all ephemeral ports land in a slice the band calls safe. So the band is
not taken on trust: ``verify_band`` reads the live range on the machine that
RUNS the tests, at the moment a test asks for a domain, and refuses to hand one
out when the band does not hold there. Reading is enough, and reading is all
that is possible: ``/proc/sys`` is mounted read-only inside a container, so the
check can see the host's setting but can never quietly "fix" it. A machine where
the file cannot be read at all is refused rather than assumed to be a default
one, since that assumption is exactly the thing the check exists to stop making.
"""

import contextlib
import errno
import os
import random
import socket
import time

import domain_coordinator

#: Where this file and its sibling scripts live, for callers that need to spawn
#: one of them.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

#: Every DDS domain a test may use. See the module docstring for the derivation.
#: Checked against the live kernel range by verify_band() before it is used, so
#: this is the band the derivation allows and never on its own the band a given
#: machine can honour.
USABLE_DOMAINS = tuple(range(1, 101)) + tuple(range(215, 232))

#: RTPS reserves [RTPS_PORT_BASE + RTPS_DOMAIN_GAIN * d, ... + GAIN - 1] for
#: domain d. Fixed by the RTPS specification, not configurable.
RTPS_PORT_BASE = 7400
RTPS_DOMAIN_GAIN = 250

#: The top of the UDP port space, which is what excludes domain 232.
MAX_UDP_PORT = 65535

#: Where the kernel publishes the ports it may hand to any process that asks for
#: one. Read at run time, on the machine that runs the tests: in CI the machine
#: that builds is not the machine that tests, so a configure-time check would be
#: reading the wrong kernel.
EPHEMERAL_RANGE_FILE = '/proc/sys/net/ipv4/ip_local_port_range'

#: Declares the ephemeral range for a machine where the file above cannot be
#: read at all. Written '<low>-<high>' or '<low> <high>'.
#:
#: When the file IS readable, this does not replace it: the two are merged to
#: the widest range, so a declaration can only ever make the check stricter.
#: That is deliberate. An override that could narrow the range would be a way to
#: silence the check, and the check has to be un-silenceable to be worth having.
EPHEMERAL_RANGE_ENV = 'MEDKIT_TEST_EPHEMERAL_PORT_RANGE'

#: The coordination port of a domain is ``PORT_BASE + domain``. Fixed by
#: ``domain_coordinator``; mirrored here so tests can reason about the lock.
PORT_BASE = domain_coordinator.impl.PORT_BASE

#: How long to keep trying before giving up, in seconds. Under ``ctest -j`` on a
#: large machine more tests can be in flight than the band has domains, and the
#: right answer for the excess is to wait for one to be handed back.
#:
#: The CMake helpers set this per test, to a fraction of that test's own CTest
#: TIMEOUT, so the wait ends and the wrapper reports while the test is still
#: alive to report it. Left at a fixed default it was routinely longer than the
#: timeout that would kill it, which made the failure message unreachable on
#: almost every test in the tree.
WAIT_TIMEOUT_ENV = 'MEDKIT_TEST_DOMAIN_WAIT'
DEFAULT_WAIT_TIMEOUT = float(os.environ.get(WAIT_TIMEOUT_ENV, '180'))

#: Environment variable a test carries to say how many domains it needs.
DOMAIN_COUNT_ENV = 'MEDKIT_TEST_DOMAINS'

#: Narrows the band this process draws from, as a comma separated list of
#: domains and inclusive ranges, for example '1-8,215'. Validated to be a subset
#: of USABLE_DOMAINS, so it can pin the allocator to part of the band but can
#: never point it at a domain the band excludes. The allocator's own tests use
#: it to work against a handful of domains they have proved they can bind,
#: rather than assuming the whole band is theirs to take.
BAND_ENV = 'MEDKIT_TEST_DOMAIN_BAND'

#: CTest label a test carries to say it creates no ROS entities at all, and so
#: needs no domain. A label rather than an environment entry on purpose: the
#: environment is inherited by everything a test starts, and a test that spawns
#: a wrapped child would hand it its own declaration.
NO_DOMAIN_LABEL = 'no_ros_domain'

#: Where the extra domains of a multi-domain test are published.
SECONDARY_ENV = 'MEDKIT_SECONDARY_DOMAINS'

_RETRY_INTERVAL = 0.05

#: Upper multiple of _RETRY_INTERVAL for the randomised backoff between
#: acquisition attempts. See _try_take for why the randomisation is load-bearing.
_RETRY_JITTER = 4


class DomainAllocationError(RuntimeError):
    """A domain could not be obtained. See the specific cases below.

    The wrappers catch this base and report it, so every refusal reaches the
    reader as a sentence rather than as a traceback. Anything raised on the
    allocation path belongs under it.
    """


class NoDomainAvailable(DomainAllocationError):
    """Raised when the whole band stayed busy for longer than the caller waited."""


class LockPortsUnusable(DomainAllocationError):
    """Raised when this machine refuses the binds the scheme is built on.

    Kept apart from NoDomainAvailable on purpose. ``domain_coordinator`` throws
    away the errno (``except OSError: pass``), so a seccomp profile answering
    EACCES, or an exhausted file descriptor limit, is indistinguishable from a
    busy band to the code above it. Reported as contention it would advise
    lowering the test parallelism, which fixes nothing and points the reader at
    the wrong thing entirely: a wrapper failure must never be readable as
    ros2_medkit being broken.
    """


class EphemeralRangeUnknown(DomainAllocationError):
    """Raised when this machine will not say which ports it hands out.

    Deliberately not "assume the Linux default and carry on". The whole band is
    derived from that range, so assuming it is guessing at the one input the
    derivation has, and the guess fails silently: the tests run, the domains
    look fine, and a node dies with an unrelated-looking bind error whenever an
    ephemeral port happens to land in a slice.
    """


class UnsafeBand(DomainAllocationError):
    """Raised when the band this run would draw from is not safe on this machine.

    Not a downgrade to whatever is still safe, and not a warning. A run on an
    unsafe domain does not fail where the mistake is; it fails later, in another
    test, as a bind error that reads like a broken package.
    """


def _parse_port_range(text, source):
    """Turn '32768 60999' or '32768-60999' into (low, high)."""
    fields = text.replace('-', ' ').split()
    if len(fields) != 2:
        raise ValueError(f'{source}: expected two port numbers, got {text.strip()!r}')
    try:
        low, high = int(fields[0]), int(fields[1])
    except ValueError:
        raise ValueError(f'{source}: {text.strip()!r} is not a pair of port numbers')
    if not 0 < low <= high <= MAX_UDP_PORT:
        raise ValueError(
            f'{source}: {low}-{high} is not a usable port range (expected '
            f'0 < low <= high <= {MAX_UDP_PORT})'
        )
    return low, high


def read_ephemeral_range(env=None, path=None):
    """Return (low, high, source) for the ports this machine may hand to anybody.

    The kernel's own answer, widened by an explicit declaration if one is set.
    Never a default: a machine that answers neither way raises
    EphemeralRangeUnknown, because a guess here is invisible when it is wrong.
    """
    if env is None:
        env = os.environ
    if path is None:
        # Read through the module attribute rather than a default argument bound
        # at definition time, so a caller can point this at another file.
        path = EPHEMERAL_RANGE_FILE
    found = []
    declared = env.get(EPHEMERAL_RANGE_ENV)
    if declared:
        low, high = _parse_port_range(declared, EPHEMERAL_RANGE_ENV)
        found.append((low, high, f'{EPHEMERAL_RANGE_ENV}={low}-{high}'))
    unreadable = None
    try:
        with open(path) as handle:
            text = handle.read()
    except OSError as error:
        unreadable = error.strerror or str(error)
    else:
        low, high = _parse_port_range(text, path)
        found.append((low, high, f'{path} = {low}-{high}'))
    if not found:
        raise EphemeralRangeUnknown(
            f'cannot tell which ports this machine hands out as ephemeral: {path} could '
            f'not be read ({unreadable}). The safe DDS domain band is derived from that '
            'range, so assuming the Linux default would be a guess about the one thing '
            'the derivation depends on, and a wrong guess shows up as a node failing to '
            'bind in some other test. Declare the range explicitly with '
            f'{EPHEMERAL_RANGE_ENV}="<low>-<high>", the value of '
            'net.ipv4.ip_local_port_range on this host, or run where the file is '
            'readable. It is readable inside a container; only writing it is not.'
        )
    low = min(entry[0] for entry in found)
    high = max(entry[1] for entry in found)
    source = ' and '.join(entry[2] for entry in found)
    if len(found) > 1:
        source += f', widened to {low}-{high}'
    return low, high, source


def domain_slice(domain):
    """Return the (first, last) UDP port RTPS reserves for a domain."""
    first = RTPS_PORT_BASE + RTPS_DOMAIN_GAIN * domain
    return first, first + RTPS_DOMAIN_GAIN - 1


def domain_is_safe(domain, low, high):
    """Say whether this domain's whole RTPS slice avoids the ephemeral range."""
    first, last = domain_slice(domain)
    if last > MAX_UDP_PORT:
        return False
    return last < low or first > high


def unsafe_domains(domains, low, high):
    """Return the domains of *domains* whose slice this machine may hand out."""
    return tuple(domain for domain in domains if not domain_is_safe(domain, low, high))


def format_band(domains):
    """Render domains the way MEDKIT_TEST_DOMAIN_BAND wants them, runs collapsed."""
    parts = []
    for domain in sorted(set(domains)):
        if parts and domain == parts[-1][1] + 1:
            parts[-1][1] = domain
        else:
            parts.append([domain, domain])
    return ','.join(
        str(low) if low == high else f'{low}-{high}' for low, high in parts
    ) or '<none>'


def verify_band(domains, env=None):
    """Refuse the band unless every domain in it is safe on THIS machine.

    Returns (low, high, source) so a caller can report what it checked against.
    """
    low, high, source = read_ephemeral_range(env)
    unsafe = unsafe_domains(domains, low, high)
    if not unsafe:
        return low, high, source
    safe = tuple(domain for domain in domains if domain_is_safe(domain, low, high))
    shown = ', '.join(
        f'{domain} ({domain_slice(domain)[0]}-{domain_slice(domain)[1]})'
        for domain in unsafe[:3]
    )
    if len(unsafe) > 3:
        shown += f', and {len(unsafe) - 3} more'
    remedy = (
        f'run against the part of the band that is still safe here by setting '
        f'{BAND_ENV}={format_band(safe)} ({len(safe)} of {len(domains)} domains)'
        if safe else
        'no domain in the band is safe on this machine, so there is no subset to fall '
        f'back to; {BAND_ENV} cannot help here'
    )
    raise UnsafeBand(
        f'this machine hands out ephemeral ports from {low}-{high} ({source}), and '
        f'{len(unsafe)} of the {len(domains)} DDS domains this run would draw from map '
        f'to a UDP slice inside it: {shown}. RTPS binds domain d at '
        f'[{RTPS_PORT_BASE} + {RTPS_DOMAIN_GAIN}d, ... + {RTPS_DOMAIN_GAIN - 1}], and an '
        'unrelated process given one of those ports first kills every node on the domain '
        'with "failed to bind to ANY:<port>: address in use". Nothing was started, '
        'because a run on an unsafe domain fails later and somewhere else. Either restore '
        'the kernel default with '
        '"sysctl -w net.ipv4.ip_local_port_range=\'32768 60999\'", or ' + remedy + '.'
    )


def parse_band(spec):
    """Turn '1-8,215' into a tuple of domains, refusing anything outside the band."""
    domains = []
    for part in spec.split(','):
        part = part.strip()
        if not part:
            continue
        if '-' in part:
            low, _, high = part.partition('-')
            try:
                bounds = range(int(low), int(high) + 1)
            except ValueError:
                raise ValueError(f'{BAND_ENV}: {part!r} is not a range of domains')
            domains.extend(bounds)
        else:
            try:
                domains.append(int(part))
            except ValueError:
                raise ValueError(f'{BAND_ENV}: {part!r} is not a domain')
    if not domains:
        raise ValueError(f'{BAND_ENV}={spec!r} names no domains')
    outside = sorted(set(domains) - set(USABLE_DOMAINS))
    if outside:
        raise ValueError(
            f'{BAND_ENV}={spec!r} names {outside}, which the safe band excludes. It can '
            'narrow the band, never widen it - see the module docstring for why those '
            'domains are not usable.'
        )
    return tuple(dict.fromkeys(domains))


def default_band(env=None):
    """Return the band this process draws from."""
    if env is None:
        env = os.environ
    spec = env.get(BAND_ENV)
    return parse_band(spec) if spec else USABLE_DOMAINS


class BandSelector:
    """Walk the band in order, from a random starting point.

    ``domain_coordinator.domain_id`` calls this for each bind attempt, so the
    only contract is "return the next candidate". Starting at a random offset
    matters under load: a fixed start means every process on the machine tries
    the same domain first and walks the same order, so the first free slot is
    contended by all of them at once.
    """

    def __init__(self, domains=USABLE_DOMAINS, start=None):
        if not domains:
            raise ValueError('BandSelector needs at least one domain to draw from')
        self._domains = tuple(domains)
        if start is None:
            start = random.randrange(len(self._domains))
        self._next = start % len(self._domains)

    def __call__(self):
        domain = self._domains[self._next]
        self._next = (self._next + 1) % len(self._domains)
        return domain


def diagnose_bind_refusal(domains, probes=3):
    """Say why binding failed, when it was not simply that the port was taken.

    ``domain_coordinator`` swallows the errno, so ask the kernel ourselves: bind
    the same kind of socket to a few of the same ports and look at what comes
    back. EADDRINUSE everywhere means the band really is contended. Anything
    else is this machine refusing the mechanism, which is a different problem
    with a different answer.

    Returns None when the evidence says ordinary contention.
    """
    seen = []
    for domain in list(domains)[:probes]:
        try:
            probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except OSError as error:
            return (
                f'this process cannot even create a socket ({error.strerror}). The DDS '
                'domain lock needs one socket per domain held; raise the open file '
                'limit for the test runner. This is the machine refusing the '
                'mechanism, not a busy band.'
            )
        try:
            probe.bind(('', PORT_BASE + domain))
        except OSError as error:
            if error.errno != errno.EADDRINUSE:
                return (
                    f'binding a DDS domain lock port was refused with '
                    f'{errno.errorcode.get(error.errno, error.errno)} '
                    f'({error.strerror}). The band is not busy, this machine will not '
                    f'let the test runner bind ports {PORT_BASE}-{PORT_BASE + 231} at '
                    'all - a sandbox or seccomp profile is the usual reason. This is '
                    'not a fault in the package under test.'
                )
            seen.append(domain)
        finally:
            probe.close()
    return None


def _release(managers):
    """Hand back every domain in *managers*.

    Always a clean exit, never the caller's exception: domain_id() catches
    OSError around its own yield, so throwing one into it would send it back
    around its retry loop instead of releasing the socket.
    """
    for manager in reversed(managers):
        manager.__exit__(None, None, None)


def _try_take(count, domains):
    """Take all *count* domains at once, or hold nothing at all.

    Returns (managers, domains) when the whole set was taken, and (None, None)
    when it was not - with nothing held on that path, which is the entire point
    of the shape.

    Taking them one at a time and keeping each while waiting for the next is
    what a caller must not do. A test asking for 4 would hold 1, 2 and 3 while
    it waited for the 4th, and two such tests can each end up holding part of
    what the other is waiting for: neither can finish, neither gives anything
    back, and both fail when their wait budgets run out. Two DOMAINS 4 launch
    tests in this repository can meet exactly that way under ctest -j.

    All-or-nothing trades that deadlock for a livelock window - two callers can
    take and release in step - which is bounded rather than permanent, and paid
    for twice over: each attempt gets a fresh randomly-started selector, and the
    caller of this function sleeps a jittered interval between attempts, so two
    callers of the same size do not stay in step.
    """
    selector = BandSelector(domains)
    managers, acquired = [], []
    try:
        for _ in range(count):
            manager = domain_coordinator.domain_id(selector)
            try:
                domain = manager.__enter__()
            except RuntimeError:
                # The band could not supply this one right now. Everything
                # already taken goes back before anything waits on it.
                _release(managers)
                return None, None
            except OSError as error:
                # socket() itself failing, which upstream creates outside its
                # own try and therefore lets escape as a bare traceback.
                _release(managers)
                raise LockPortsUnusable(
                    f'could not create the socket that holds a DDS domain: '
                    f'{error.strerror}. Raise the open file limit for the test runner. '
                    'This is the machine refusing the mechanism, not a fault in the '
                    'package under test.'
                ) from error
            managers.append(manager)
            acquired.append(domain)
    except BaseException:
        _release(managers)
        raise
    return managers, acquired


@contextlib.contextmanager
def hold_domains(count=1, domains=None, wait_timeout=None, announce=None):
    """Hold *count* distinct DDS domains for the duration of the context.

    Acquisition is all-or-nothing: while this waits it holds nothing, so a
    caller waiting for its set can never be the reason another caller cannot
    complete its own. See _try_take.

    The domains are released when the context exits, and by the kernel closing
    the sockets if the process dies instead.
    """
    if domains is None:
        domains = default_band()
    if count < 1:
        raise ValueError(f'a test needs at least one domain, asked for {count}')
    verify_band(domains)
    if wait_timeout is None:
        wait_timeout = DEFAULT_WAIT_TIMEOUT
    if announce is None:
        def announce(message):
            print(f'[medkit-domain] {message}', flush=True)

    deadline = time.monotonic() + wait_timeout
    waited_from = None
    while True:
        managers, acquired = _try_take(count, domains)
        if managers is not None:
            break
        refusal = diagnose_bind_refusal(domains)
        if refusal is not None:
            raise LockPortsUnusable(refusal)
        if waited_from is None:
            waited_from = time.monotonic()
            announce(
                f'could not take {count} of the {len(domains)} DDS domains in the band '
                f'at once, so this holds none of them and waits for the band to free up '
                f'(up to {max(0.0, deadline - waited_from):.0f}s). Holding a partial set '
                'while waiting is what would block another test.'
            )
        if time.monotonic() >= deadline:
            raise NoDomainAvailable(
                f'no free ROS_DOMAIN_ID after waiting '
                f'{time.monotonic() - waited_from:.0f}s: {count} of the {len(domains)} '
                'domains in the safe band were never free at the same moment, every '
                'attempt found one of them held by another test. Nothing may run without '
                'a domain, so this test was not started - running on the default domain 0 '
                'would make it see every node on the machine. Raise '
                'MEDKIT_TEST_DOMAIN_WAIT if the machine is simply slow, or lower the test '
                'parallelism.'
            )
        # Jittered: two callers that keep taking and releasing in lockstep would
        # be a livelock, and a fixed interval is what keeps them in lockstep.
        time.sleep(random.uniform(_RETRY_INTERVAL, _RETRY_INTERVAL * _RETRY_JITTER))
    try:
        yield acquired
    finally:
        _release(managers)


def apply_to_environ(domains, env=None):
    """Publish the held domains the way every test reads them."""
    if env is None:
        env = os.environ
    env['ROS_DOMAIN_ID'] = str(domains[0])
    if len(domains) > 1:
        env[SECONDARY_ENV] = ','.join(str(domain) for domain in domains[1:])
    else:
        env.pop(SECONDARY_ENV, None)
    # The count is an instruction to this wrapper, not to anything it starts.
    # Left in place, a test that runs a wrapped command of its own would inherit
    # its parent's request and ask for that many domains again.
    env.pop(DOMAIN_COUNT_ENV, None)


def requested_domain_count(env=None):
    """Return how many domains the caller's environment asks for."""
    if env is None:
        env = os.environ
    raw = env.get(DOMAIN_COUNT_ENV, '1')
    try:
        count = int(raw)
    except ValueError:
        raise SystemExit(
            f'{DOMAIN_COUNT_ENV}={raw!r} is not a number. It is set by the CMake '
            'helpers in ROS2MedkitTestDomain.cmake and must be a count of domains.'
        )
    if count < 1:
        raise SystemExit(
            f'{DOMAIN_COUNT_ENV}={count} reached a test that goes through the domain '
            'wrapper. A test needs at least one domain, or it must not be wrapped at '
            'all - medkit_test_needs_no_domain() is how a test says it needs none.'
        )
    return count
