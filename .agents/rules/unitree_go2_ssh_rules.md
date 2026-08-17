# Unitree GO2 SSH Operational & Communication Rules

1. **Pre-Explanation Mandate**:
   - ALWAYS explain what command/tool you plan to execute and WHY before executing it.
   - Never run shell commands or tool calls silently without prior user-facing rationale.

2. **NetBird VPN & Network Preservation**:
   - The user connects to Jetson Orin NX via NetBird VPN (`wt0`: 100.96.204.119).
   - NEVER alter network interface configurations (`/etc/network/`), IP routes, iptables, or system DNS.

3. **SSH Terminal Direct Output**:
   - The user is working in an SSH terminal without an artifact GUI browser.
   - Always print full markdown text directly in the chat output instead of relying only on artifact file links.

4. **Credentials & Device State**:
   - GO2 SSH password: `admin` (changed from default `123`).
   - Jetson Orin NX IP: `192.168.123.99`. GO2 Motion Controller IP: `192.168.123.13`.
