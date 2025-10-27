---
title: "Jetson Boot Failure — eMMC Full"
date: "2025-10-23"
summary: "Troubleshooting guide for resolving Jetson boot failures caused by full eMMC storage, including investigation steps and preventive measures."
writer: "Chaerin An"
keywords: "Jetson Xavier, eMMC, Troubleshooting, Storage Management, Boot Failure"
---

### **Summary**

Our Jetson failed to boot because the **main system partition** (`/dev/mmcblk0p1`) ran out of space. This partition holds Ubuntu and all system directories under `/`, except for what's mounted on the SSD (`/ssd`).

When the eMMC reached 100% usage, the OS couldn't start properly.

---

### Investigation History

```bash
df -h
```

It was **100% full**, and after removing `/tmp` folders, it went down to **98%**, so we were able to boot up the Jetson.

98% was still too high, so we checked which folders were using the most memory:

```bash
sudo du -h --max-depth=1 / | sort -hr | head -20
```

```
26G     /
7.6G    /home
7.6G    /usr
1.4G    /usr/src
1.2G    /var
1.1G    /opt
128M    /boot
20M     /etc
16M     /root
8.0K    /srv
4.0K    /mnt
4.0K    /media
```

Found that files under `/usr` folder were saved on eMMC, so we decided to delete some files there.

---

```bash
sudo du -h --max-depth=1 /usr | sort -hr | head -20
```

```
7.6G    /usr
5.2G    /usr/lib
1.4G    /usr/src
354M    /usr/share
270M    /usr/lib/libreoffice
237M    /usr/lib/firefox
219M    /usr/lib/thunderbird
162M    /usr/lib/gcc
117M    /usr/lib/modules
82M     /usr/lib/snapd
```

---

```bash
sudo du -h --max-depth=1 /usr/lib | sort -hr | head -20
```

`lib` was the biggest one, but it had core Ubuntu system files, so we couldn't reduce much.

We removed Thunderbird and some other programs that weren't being used.

---

Then we checked the home directory:

```bash
sudo du -h --max-depth=1 /home/ias | sort -hr | head -20
```

```
7.6G	/home/ias
1.8G	/home/ias/.local
1.6G	/home/ias/.vscode-server
1.5G	/home/ias/.platformio
564M	/home/ias/.cache
559M	/home/ias/.cursor-server
363M	/home/ias/.mozilla
354M	/home/ias/librealsense
266M	/home/ias/rtl8812au
228M	/home/ias/mecanum_gym
129M	/home/ias/.nvm
111M	/home/ias/ngc-cli
53M	/home/ias/8812au-20210820
48M	/home/ias/.nx
40M	/home/ias/Documents
35M	/home/ias/src
21M	/home/ias/python-clients
13M	/home/ias/.thunderbird
9.5M	/home/ias/Rover
5.6M	/home/ias/Pictures
```

Found 1.6G cache files for VSCode that get created whenever we SSH into the rover through VSCode.

Same for Cursor IDE as well.

Didn't mention here, but we also removed more cache and temporary files, and the usage went down to **86%**.

---

```bash
ias@ubuntu:~/Documents$ df -h
```

```
Filesystem      Size  Used Avail Use% Mounted on
/dev/mmcblk0p1   28G   23G  3.8G  86% /
none             16G     0   16G   0% /dev
tmpfs            16G   10M   16G   1% /dev/shm
tmpfs           3.1G   20M  3.1G   1% /run
tmpfs           5.0M  4.0K  5.0M   1% /run/lock
tmpfs            16G     0   16G   0% /sys/fs/cgroup
/dev/loop1       45M   45M     0 100% /snap/snapd/25205
/dev/loop0       43M   43M     0 100% /snap/snapd/24787
/dev/loop3       50M   50M     0 100% /snap/core18/2956
/dev/loop2       50M   50M     0 100% /snap/core18/2950
/dev/nvme0n1    916G  388G  483G  45% /ssd
tmpfs           3.1G   44K  3.1G   1% /run/user/1000
```

---

### **Root Cause**

- The internal **eMMC (28 GB)** partition contained both the operating system and several large files under `/usr` and `/home`.
- Heavy applications (like CUDA, TensorRT, and RealSense SDK) and user caches accumulated over time.
- Some processes were still writing temporary files to `/`, even though we were using the SSD for data.
- Docker images were too big (~13GB) so we investigated them as well. However, docker images were saved in `/ssd` so didn't affect to this issue.

---

### **Actions Taken**

1. **Freed up space manually**
   - Removed unused packages and apps: `libreoffice`, `thunderbird`
   - Deleted old Wi-Fi driver source folders (`rtl8812au`, etc.).
   - Cleaned up caches and IDE files (`~/.vscode-server`, `.cache`, `.cursor-server`).
2. **Verified system health**
   - Checked disk usage with `df -h` and confirmed `/dev/mmcblk0p1` usage dropped.
   - Ensured Docker now writes to `/ssd` and not to eMMC.
   - Confirmed Realtek Wi-Fi driver is installed via DKMS (`modinfo 8812au`) so old sources could be safely removed.

---

### **Resolution**

After cleanup, the eMMC usage decreased significantly, allowing the Jetson to boot normally. The system now uses the SSD for Docker, rosbags, and large project data, keeping the eMMC dedicated to OS files only.

---

### **Next Steps / Prevention**

- Save heavy data (e.g., rosbags, workspaces) to `/ssd` by default. Never save them under `/usr` .
- Run `df -h` periodically to monitor eMMC usage.
- Optionally, document all default storage mount points and data paths for future team members.
