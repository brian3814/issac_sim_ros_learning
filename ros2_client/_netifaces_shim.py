"""A minimal stand-in for the ``netifaces`` package.

Isaac Sim bundles ros2cli but not ``netifaces``, which ``ros2cli.daemon``
imports unconditionally. Without it every ``ros2 <thing> <verb>`` fails with a
confusing ``KeyError: 'list'`` -- the verb is found in the entry points but
cannot be imported, and ros2cli reports the miss rather than the cause.

Rather than pip-installing into your Isaac Sim (this project installs
nothing), we register a tiny replacement in ``sys.modules`` before ros2cli is
imported. Only three names are ever used:

    netifaces.interfaces()          -> list of interface names
    netifaces.ifaddresses(name)     -> {AF_INET: [{'addr': ip}, ...]}
    netifaces.AF_INET

and both call sites only want "which IPv4 addresses does this machine have",
to notice network changes and to accept local XML-RPC connections. The stdlib
``socket`` module answers that well enough.
"""

from __future__ import annotations

import socket
import sys
import types

AF_INET = int(socket.AF_INET)
AF_INET6 = int(socket.AF_INET6)
AF_LINK = -1000  # not discoverable without platform APIs; nothing here uses it


def _local_ipv4_addresses() -> "list[str]":
    addrs = ["127.0.0.1"]
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = str(info[4][0])
            if ip not in addrs:
                addrs.append(ip)
    except OSError:
        pass
    # A UDP connect() to a public address does not send anything, but it does
    # make the OS pick the outbound interface -- the reliable way to learn the
    # address other machines would reach us on.
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(("8.8.8.8", 80))
            ip = probe.getsockname()[0]
            if ip not in addrs:
                addrs.append(ip)
        finally:
            probe.close()
    except OSError:
        pass
    return addrs


def interfaces() -> "list[str]":
    return ["lo"] + ["if%d" % i for i in range(len(_local_ipv4_addresses()) - 1)]


def ifaddresses(interface: str) -> dict:
    addrs = _local_ipv4_addresses()
    if interface == "lo":
        return {AF_INET: [{"addr": "127.0.0.1", "netmask": "255.0.0.0"}]}
    try:
        idx = int(interface[2:]) + 1
    except (ValueError, IndexError):
        return {}
    if idx >= len(addrs):
        return {}
    return {AF_INET: [{"addr": addrs[idx], "netmask": "255.255.255.0"}]}


def install() -> bool:
    """Register this module as ``netifaces`` if the real one is absent.

    Returns True if the shim was installed, False if the real package is
    already importable (in which case we stay out of the way).
    """
    if "netifaces" in sys.modules:
        return False
    try:
        import netifaces  # noqa: F401  -- the real thing is present
        return False
    except ImportError:
        pass

    module = types.ModuleType("netifaces")
    for name, value in (
        ("AF_INET", AF_INET),
        ("AF_INET6", AF_INET6),
        ("AF_LINK", AF_LINK),
        ("interfaces", interfaces),
        ("ifaddresses", ifaddresses),
        ("version", "0.0.0-isaac-shim"),
    ):
        setattr(module, name, value)
    sys.modules["netifaces"] = module
    return True
