# Working on the robot remotely

The laptop is the robot's brain (gate H0) and rides on the chassis with the lid shut. That makes
SSH the only way to touch it once it is driving.

## Set it up (once)

```bash
sudo bash bringup/remote_access.sh
# or, for a shorter name to type:
sudo bash bringup/remote_access.sh --hostname utp
```

Installs `openssh-server`, `mosh`, `tmux`, `avahi`; stops the machine suspending for any reason;
turns off WiFi power-save; and configures sshd for **key auth only**.

## Three things that are specific to a robot, not a desk

**The lid is shut.** Default logind suspends on lid close, which strands the robot mid-run with no
fix but walking over and opening it. The script sets `HandleLidSwitch=ignore` *and* masks the sleep
targets — both are needed, because desktop power managers can request a suspend that logind alone
would not.

**Nobody touches the keyboard.** Driving a robot is not keyboard input, so default idle handling
will suspend a machine that is very much busy. `IdleAction=ignore`.

**It moves.** Roaming between access points drops TCP. WiFi power-save makes it worse by parking
the radio between packets, which on a moving robot is indistinguishable from a roaming problem.
Power-save off, and:

- **Use `mosh`, not `ssh`, while driving.** It survives roams and IP changes; ssh does not.
- **Always work inside `tmux`.** A dropped link kills anything not in a session — including a
  mapping run you are forty minutes into.

```bash
tmux new -s utp        # start
tmux attach -t utp     # after a drop
```

## Finding it again

The address changes: campus DHCP re-leases whenever it likes. With the lid shut there is no screen
to read it off, so it is written to disk at every bringup — `bringup/env.sh` stamps
`.last_address` each time it is sourced.

```bash
bash bringup/whereami.sh      # print every address, and record them
cat .last_address             # what it was at the last bringup
ssh weim@<hostname>.local     # mDNS, if the network passes multicast
```

## If SSH does not work at all

**Campus WiFi very often has client isolation** — clients can reach the internet but not each
other. That blocks SSH between two machines on the same SSID no matter how sshd is configured, and
it looks exactly like a firewall problem on the robot.

Test it **before** you rely on it, from the machine you intend to connect from, while the robot is
still within reach of a keyboard.

If it is blocked, the fix is an overlay network rather than more sshd configuration — Tailscale or
ZeroTier both give the machines a stable private address that works regardless of client isolation,
and survive the robot changing networks entirely. That is also the answer if you ever want to reach
it from off campus.

## Security note

`remote_access.sh` refuses to enable password authentication. This machine sits on a `/16` campus
network, not a private lab LAN, and a password-authenticated SSH server there will be found and
attacked within hours. Add your public key to `~/.ssh/authorized_keys` from the keyboard before you
need remote access — the script prints the exact commands.
