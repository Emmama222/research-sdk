# Notes: `multicast_interface_ip` in `network_input.yaml`

File: `src/research_sdk/config/network_input.yaml`, line 3.

## What it's for

`multicast_interface_ip` tells your OS **which network adapter** to use when
joining the SSL multicast groups (vision, tracker, referee packets). It is
not "your app's identity" — it's "which NIC do I attach to this multicast
group on."

## Why it matters even on one machine

- Multicast joining happens at the OS/adapter level (`IP_ADD_MEMBERSHIP`),
  not the application level. The OS needs to know which physical/virtual
  adapter should send the "I want to receive traffic for group 224.5.23.2"
  request.
- A Windows machine almost always has more than one network adapter, even
  when you think you're "just using localhost": VPN clients, WSL's virtual
  switch, Hyper-V/VirtualBox virtual adapters, Docker's bridge, etc.
- `0.0.0.0` means "OS, pick an interface for me" — based on routing-table
  priority, not on which adapter can actually see multicast traffic. This
  guess is frequently wrong when extra virtual adapters exist.
- Multicast doesn't follow normal unicast/internet routing. Regular traffic
  reliably resolves `0.0.0.0` to your main internet-facing adapter; the
  interface-selection logic for multicast group membership is separate and
  less reliable on Windows.
- Loopback multicast is inconsistent: even with grSim on `127.0.0.1`,
  packets only loop back if the group was joined on the correct adapter with
  multicast loopback enabled — if the OS joins on the wrong (virtual)
  adapter, you get silence even though sender and receiver are the same box.

## Practical takeaway

- Same machine (grSim + this code both local): try `0.0.0.0` first — often
  fine since there's no real LAN to route across.
- If no vision/tracker packets show up, switch to your real IPv4 (from
  `ipconfig`, the adapter actually on the same network as the vision
  system/grSim) — that's why the file keeps both, one commented out as a
  fallback:
  ```yaml
  multicast_interface_ip: 10.105.229.208 #0.0.0.0
  ```

## Analogy used in conversation

Think of your laptop as having several "doors" to the outside world (Wi-Fi,
Ethernet, VPN, virtual adapters). grSim/vision shouts (multicasts) onto
whichever network it's plugged into — the shout only travels on that one
network. `0.0.0.0` = "OS, pick a door for me" (sometimes wrong door).
Hardcoding your real IPv4 = "specifically listen at the door connected to
the network grSim/vision is shouting on."
