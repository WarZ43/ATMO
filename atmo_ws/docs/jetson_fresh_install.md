# Fresh Jetson install (Orin Nano devkit, JetPack 6.2.x)

Everything the ATMO stack needs on a newly flashed Jetson, in install order.
Written 2026-08-17 for the board swap from the old Jetson (JetPack 6.0, USB
WiFi dongle) to the Orin Nano Developer Kit (onboard PCIe WiFi). The RL stack
is deliberately light: ROS 2 Humble, the XRCE agent, this workspace, and
numpy. No torch, no CUDA, no acados (those imports are the legacy MPC
controllers, which the RL stack never touches).

## 0. At OS first-boot setup

- Username **m4**, hostname m4. Every doc, script default and ssh command in
  this repo assumes `/home/m4`.
- Enable the OpenSSH server if the installer offers it; else step 2.

## 1. WiFi to the robot router

```bash
nmcli device wifi connect "<robot-router-ssid>" password "<psk>"
nmcli connection modify "<robot-router-ssid>" connection.autoconnect yes
ip -4 -brief addr    # record the new IP; the old 192.168.0.44 lease is gone
```

The router has no internet. For the apt/pip steps below, either temporarily
join a network that has internet, or use a phone hotspot, then switch back.
(The laptop can also share its campus connection over the Ethernet port.)

Acceptance test before trusting the link — this is what the old dongle
failed: pull a 60 MB file from the laptop and time it. Seconds = good,
minutes = stop and diagnose before building anything on top.

**Mandatory WiFi settings, learned 2026-08-17** (a scan storm took the radio
off-channel every ~7 s and punched 2-6 s holes in the mocap stream while
signal looked perfect — see session_state.md):

```bash
sudo nmcli c modify "<ssid>" 802-11-wireless.bssid <5GHz-AP-BSSID>  # pin, no roaming
sudo nmcli c modify "<ssid>" 802-11-wireless.powersave 2            # disable
# autoconnect OFF on every other saved wifi profile
printf '[device]\nwifi.scan-rand-mac-address=no\n' | \
    sudo tee /etc/NetworkManager/conf.d/90-no-scan-rand.conf
sudo systemctl restart NetworkManager
```

Verify with `iw event` for 45 s: zero `scan started` lines while connected.
Then the streaming acceptance: `scripts/fake_vrpn_publisher.py` on the laptop,
`hardware_optitrack_check.py --mode rate --duration 120` on the Jetson —
worst gap under 40 ms, zero dropouts, repeated for 10 minutes.

If the dongle is the Archer T2U Plus (RTL8811AU), the driver is out-of-tree:

```bash
git clone --depth 1 https://github.com/morrownr/8821au-20210708.git
cd 8821au-20210708 && make -j$(nproc) && sudo make install && sudo modprobe 8821au
```

## 2. Base packages

```bash
sudo apt update
sudo apt install -y openssh-server git tmux curl python3-pip \
    build-essential cmake
sudo usermod -aG dialout m4     # serial ports; log out/in to take effect
```

## 3. ROS 2 Humble

Standard apt install (Ubuntu 22.04):

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools python3-colcon-common-extensions
```

`ros-base` (no desktop/GUI) is enough; add `ros-humble-rviz2` only if you want
visualization on the robot itself (the laptop is the better place for it).

## 4. Python deps

```bash
pip install numpy pytest
```

The old Jetson ran numpy 2.2.6; the code is numpy-2 clean (the port already
purged `.ptp()`/`np.float_`/etc.). Do NOT `pip install --user setuptools` —
a user-site setuptools newer than Ubuntu's packaging module breaks
`colcon build` with `canonicalize_version() got an unexpected keyword
argument` (measured on the laptop, 2026-08-17).

## 5. Micro XRCE-DDS Agent

The bridge between PX4 and ROS 2 (`interface.sh` expects `MicroXRCEAgent` on
PATH). Built from source:

```bash
cd ~ && git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig
```

The snap (`sudo snap install micro-xrce-dds-agent --edge`) also works but
installs as `micro-xrce-dds-agent`; symlink it to `MicroXRCEAgent` if used.

## 6. The workspace

```bash
cd ~ && git clone git@github.com:WarZ43/ATMO.git ATMO_rl
cd ATMO_rl && git checkout m4-lessons-port
cd atmo_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install
bash scripts/run_unit_tests.sh    # expect 99 passed, 1 skipped
```

`px4_msgs` in this tree is the FC-matched set (PX4 v1.17.0, git 8874d533).
Do not swap it for upstream.

## 7. The policy

```bash
mkdir -p ~/policies
scp <laptop>:~/ATMO/atmo_ws/policies/atmo_combined_stage1_policy.npz ~/policies/
md5sum ~/policies/atmo_combined_stage1_policy.npz   # compare against source
```

Verified 2026-08-17 on the laptop: loads as obs=529/act=7, matches the
contract (obs 529, task 5) and the runtime config, 2000-random-obs forward
pass all finite. Source checkpoint
`last_atmo_combined_stage1_ep_1800_rew_890.5606.pth`, sha256 `53e4b221...`
(embedded in the archive; the node logs it on load).

## 8. ~/.bashrc

Append (keep it minimal — the OLD bashrc's habit of sourcing the firmware
autonomy stack is what hid the domain mismatch on m4):

```bash
source /opt/ros/humble/setup.bash
export ATMO_ROS_DOMAIN_ID=0     # must equal PX4's UXRCE_DDS_DOM_ID
export ATMO_RL_OFFBOARD_CHANNEL=-1   # single-gate airframe (T14SG)
```

Do NOT set ROS_DOMAIN_ID/CYCLONEDDS_URI/ROS_DISCOVERY_SERVER globally;
`scripts/atmo_env.sh` handles those per shell.

## 9. Tools

```bash
mkdir -p ~/tools
scp <laptop-or-old-card>:~/tools/mavlink_shell.py ~/tools/
pip install pymavlink pyserial    # mavlink_shell deps
```

## 10. Things that MUST be re-measured on the new board

Carried state that does not survive a carrier-board change:

- **RoboClaw by-path names.** by-path names the USB SOCKET. The tilt/drive
  assignment (old board: 2.1 = TILT, 2.2 = DRIVE) is invalid until re-measured
  by commanding one motor at a time (read-only identification does not work —
  both boards report identically). Update `ATMO_TILT_ROBOCLAW` /
  `ATMO_DRIVE_ROBOCLAW` or the defaults in `roboclaw_safety.py` and
  `shadow_test.py`.
- **The CP2102 (TELEM2) device path** for `interface.sh` (`/dev/ttyUSB0` on
  the old board; confirm with `ls /dev/serial/by-path/`).
- **WiFi interface name** (`wlan0` vs `wlP1p1s0` etc.) if anything scripts it.
- **The Jetson's IP** on the robot router — new MAC, new DHCP lease. Update
  the laptop-side ssh targets and `fastdds_super_client.xml` notes.

## 11. Things deliberately NOT installed

- torch — the Jetson runs the numpy actor; export happens off-robot.
- acados / acados_template — legacy MPC controllers only.
- CUDA/cuDNN/TensorRT SDK components — nothing uses them.
- vrpn_mocap — lives on the LAPTOP (`scripts/run_mocap_laptop.sh`), by design.
- The CATMO / ws_athenaIII autostart units. Check none exist:
  `systemctl --user list-units --type=service --all | grep -i atmo` must be
  empty on a fresh install, and keep it that way.
