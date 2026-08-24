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
   - Never store or repeat SSH/sudo passwords, API keys, or tokens in repository
     instructions. Use an approved credential mechanism and redact diagnostics.
   - Jetson Go2-LAN IP is configured as `192.168.123.99`; the audited Go2 DDS
     peer is `192.168.123.161`. Treat both as configuration, not live health,
     and revalidate before use.

5. **Authoritative Project Memory**:
   - Read `/home/unitree/go2_ws_antarctica/AGENTS.md` and
     `/home/unitree/go2_ws_antarctica/docs/CODEX_PROJECT_MEMORY.md` before
     deployment or real-robot work. Those reviewed files supersede older
     completion dashboards and quick-run instructions.
