#!/usr/bin/python3
from __future__ import annotations

import argparse
import ipaddress
import json
import os
import socket
import struct
import subprocess
import sys
import time
from dataclasses import dataclass


DEFAULT_AUBO_MAC = "CC:82:7F:A3:E6:2E"


@dataclass(frozen=True)
class InterfaceNetwork:
    name: str
    mac: bytes
    source_ip: ipaddress.IPv4Address
    network: ipaddress.IPv4Network


def normalize_mac(value: str) -> str:
    return "".join(ch for ch in value.lower() if ch in "0123456789abcdef")


def format_mac(value: bytes) -> str:
    return ":".join(f"{byte:02x}" for byte in value)


def parse_mac(value: str) -> bytes:
    normalized = normalize_mac(value)
    if len(normalized) != 12:
        raise ValueError(f"Invalid MAC address: {value}")
    return bytes.fromhex(normalized)


def run_ip_json() -> list[dict]:
    output = subprocess.check_output(["ip", "-j", "addr", "show", "up"], text=True)
    return json.loads(output)


def discover_networks(interface_filter: str | None) -> list[InterfaceNetwork]:
    networks: list[InterfaceNetwork] = []
    for entry in run_ip_json():
        name = entry.get("ifname", "")
        if interface_filter and name != interface_filter:
            continue
        link_mac = entry.get("address")
        if not link_mac:
            continue
        try:
            mac = parse_mac(link_mac)
        except ValueError:
            continue
        for addr in entry.get("addr_info", []):
            if addr.get("family") != "inet":
                continue
            local = ipaddress.IPv4Address(addr["local"])
            prefix = int(addr["prefixlen"])
            network = ipaddress.IPv4Network(f"{local}/{prefix}", strict=False)
            networks.append(InterfaceNetwork(name, mac, local, network))
    return networks


def arp_packet(source_mac: bytes, source_ip: ipaddress.IPv4Address, target_ip: ipaddress.IPv4Address) -> bytes:
    ethernet = b"\xff" * 6 + source_mac + struct.pack("!H", 0x0806)
    arp = struct.pack(
        "!HHBBH6s4s6s4s",
        1,
        0x0800,
        6,
        4,
        1,
        source_mac,
        source_ip.packed,
        b"\x00" * 6,
        target_ip.packed,
    )
    return ethernet + arp


def read_replies(sock: socket.socket, target_mac: bytes, deadline: float) -> tuple[str, str] | None:
    target_normalized = normalize_mac(format_mac(target_mac))
    while time.monotonic() < deadline:
        try:
            frame = sock.recv(65535)
        except BlockingIOError:
            time.sleep(0.002)
            continue
        if len(frame) < 42 or frame[12:14] != b"\x08\x06":
            continue
        try:
            htype, ptype, hlen, plen, opcode = struct.unpack("!HHBBH", frame[14:22])
        except struct.error:
            continue
        if (htype, ptype, hlen, plen, opcode) != (1, 0x0800, 6, 4, 2):
            continue
        sender_mac = frame[22:28]
        sender_ip = socket.inet_ntoa(frame[28:32])
        if normalize_mac(format_mac(sender_mac)) == target_normalized:
            return sender_ip, format_mac(sender_mac)
    return None


def scan_network(
    network: InterfaceNetwork,
    target_mac: bytes,
    *,
    timeout: float,
    rate: int,
    max_hosts: int,
) -> tuple[str, str] | None:
    hosts = [host for host in network.network.hosts() if host != network.source_ip]
    if len(hosts) > max_hosts:
        print(
            f"skip {network.name} {network.network}: {len(hosts)} hosts exceeds --max-hosts={max_hosts}",
            file=sys.stderr,
        )
        return None

    print(f"scan {network.name} {network.network} from {network.source_ip} ({len(hosts)} hosts)")
    try:
        sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0806))
        sock.bind((network.name, 0))
        sock.setblocking(False)
    except OSError as exc:
        print(f"skip {network.name} {network.network}: {exc}", file=sys.stderr)
        return None

    interval = 1.0 / max(rate, 1)
    next_send = time.monotonic()
    for host in hosts:
        now = time.monotonic()
        if now < next_send:
            time.sleep(next_send - now)
        try:
            sock.send(arp_packet(network.mac, network.source_ip, host))
        except OSError as exc:
            print(f"skip {network.name} {network.network}: {exc}", file=sys.stderr)
            sock.close()
            return None
        next_send = time.monotonic() + interval
        found = read_replies(sock, target_mac, time.monotonic())
        if found:
            sock.close()
            return found

    found = read_replies(sock, target_mac, time.monotonic() + timeout)
    sock.close()
    return found


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Find an Aubo controller IP by MAC address using ARP discovery."
    )
    parser.add_argument("--mac", default=DEFAULT_AUBO_MAC, help="target MAC address")
    parser.add_argument("--interface", help="scan only this network interface")
    parser.add_argument(
        "--include-large",
        action="store_true",
        help="allow scanning large networks such as 169.254.0.0/16",
    )
    parser.add_argument("--timeout", type=float, default=2.0, help="reply wait after each subnet")
    parser.add_argument("--rate", type=int, default=3000, help="ARP requests per second")
    args = parser.parse_args()

    target_mac = parse_mac(args.mac)
    max_hosts = 70000 if args.include_large else 4096

    if os.geteuid() != 0:
        print("This script needs raw-socket access. Run it with sudo.", file=sys.stderr)
        return 1

    networks = discover_networks(args.interface)
    if not networks:
        print("No UP IPv4 interfaces found.", file=sys.stderr)
        return 1

    for network in networks:
        found = scan_network(
            network,
            target_mac,
            timeout=args.timeout,
            rate=args.rate,
            max_hosts=max_hosts,
        )
        if found:
            ip, mac = found
            print(f"FOUND {ip} {mac} interface={network.name}")
            return 0

    print(f"NOT FOUND {format_mac(target_mac)}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
