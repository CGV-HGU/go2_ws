#!/usr/bin/env python3
import subprocess
import time

password = "admin"

def run_sudo(cmd_list):
    # Prepare the command to receive password via stdin
    full_cmd = ["sudo", "-S"] + cmd_list
    proc = subprocess.Popen(full_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    stdout, stderr = proc.communicate(input=password + "\n")
    print(f"Command: {' '.join(cmd_list)}")
    print(f"Exit Code: {proc.returncode}")
    if stdout.strip():
        print(f"Stdout: {stdout.strip()}")
    if stderr.strip():
        print(f"Stderr: {stderr.strip()}")
    print("-" * 40)
    return proc.returncode

print("1. Copying udev rules...")
run_sudo(["cp", "/etc/udev/rules.d/99-realsense-libusb.rules.bak", "/etc/udev/rules.d/99-realsense-libusb.rules"])

print("2. Reloading udev rules...")
run_sudo(["udevadm", "control", "--reload-rules"])

print("3. Triggering udev...")
run_sudo(["udevadm", "trigger"])

print("4. Soft-resetting USB port 2-2...")
# Toggle authorization
run_sudo(["sh", "-c", "echo 0 > /sys/bus/usb/devices/2-2/authorized"])
time.sleep(2)
run_sudo(["sh", "-c", "echo 1 > /sys/bus/usb/devices/2-2/authorized"])

print("Waiting 5 seconds for USB device to settle...")
time.sleep(5)
