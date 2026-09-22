# Setup Elsabot on Jetson AGX running Jetpack 7.2.1


### Jetpack version

```
cat /etc/nv_tegra_release
# R39 (release), REVISION: 2.1, GCID: 46758480, BOARD: generic, EABI: aarch64, DATE: Fri Aug  7 05:54:22 AM UTC 2026
```

## Steps

1\. Used directions for fresh install onto AGX Orin (https://docs.nvidia.com/jetson/agx-orin-devkit/user-guide/latest/quick_start.html).  JP 7.2.x supports using a bootable usb stick (vs. SDKManager) that updates the firmware and then install linux.  After all the steps are complete, the firmware, kernel (ubuntu), and jetpack are installed.

2\. Installed vscode.

3\. Installed ssh server.  Copied .ssh files from previous install.

4\. Copied changes from previous .bashrc (alias, tokens, etc).

5\. Copied git config.

6\. Built kernel driver modules needed for Elsabot hardware (CH341, CP2112 HID, TOUCHSCREEN_MTOUCH).

For Kernel version 39.2.1:
See https://docs.nvidia.com/jetson/archives/r39.2/DeveloperGuide/SD/Kernel/KernelCustomization.html

a. Downloaded kernel source:

From page: https://developer.nvidia.com/embedded/jetpack/downloads/archive-7.2.1

Downloaded this file:
https://developer.nvidia.com/downloads/embedded/L4T/r39_Release_v2.1/release/Jetson_Linux_R39.2.1_aarch64.tbz2

b. Untarred

```
mkdir ~/build_kernel
cd ~/build_kernel
tar -xjf  <the dl'ed file>
```

d. Synced source using:

```
cd Linux_for_Tegra/source
./source_sync.sh -k -t  jetson_39.2.1
```

e. Installed build tools

```
sudo apt install build-essential bc flex bison libssl-dev zstd
sudo apt install ncurses-dev
```

f. Edited kernel config

Searched for ch341, cp2112, TOUCHSCREEN_MTOUCH and set each to 'm'

```
make ARCH=arm64 -C kernel/kernel-noble LOCALVERSION=-tegra menuconfig oldconfig
```

(saved to .config)

g. Ran build

```
make KERNEL_DEF_CONFIG=oldconfig -C kernel
```

h. Copied modules:

From directory:  ~/build_kernel/Linux_for_Tegra/source

```
sudo cp kernel/kernel-noble/drivers/usb/serial/ch341.ko /lib/modules/6.8.12-1021-tegra/kernel/drivers/usb/serial/
sudo cp kernel/kernel-noble/drivers/hid/hid-cp2112.ko /lib/modules/6.8.12-1021-tegra/kernel/drivers/hid/
sudo cp kernel/kernel-noble/drivers/input/touchscreen/mtouch.ko /lib/modules/6.8.12-1021-tegra/kernel/drivers/input/touchscreen/
```

And updated deps:

```
sudo depmod -a
```

7\. Added ebotagx user to 'i2c' group

```
sudo usermod -aG i2c ebotagx
```

Logged out and back in. Checked for usb-to-i2c device:

```
sudo apt install i2c-tools
i2cdetect -l
```

verified it found the CP2112:

```
i2c-9	i2c       	CP2112 SMBus Bridge on hidraw3  	I2C adapter
```

then ran:

```
i2cdetect -r -y 9     (9 is the bus shown above)
```

And confirmed it found the I2C devices on the bus

8\.  Installed jtop

See https://pypi.org/project/jetson-stats/

Used option 4:

```
sudo -v
curl -LsSf https://raw.githubusercontent.com/rbonghi/jetson_stats/master/scripts/upgrade-jtop.sh | bash
```

Added user to jtop group:

```
sudo usermod -aG jtop $USER
```

Logged out/in and then confirmed jtop without pw needed.


9\. Installed docker (was already installed but needed nvidia runtime setup).

```
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl daemon-reload
sudo systemctl restart docker
```

Setting NVIDIA as the default runtime ensures that containers can leverage the Jetson GPU.

Used jq to modify the file (although you could do it with an editor).

```
sudo apt install -y jq
```

Updated daemon.json to use nvidia as the default runtime

```
sudo jq '. + {"default-runtime": "nvidia"}' /etc/docker/daemon.json | sudo tee /etc/docker/daemon.json.tmp
sudo mv /etc/docker/daemon.json.tmp /etc/docker/daemon.json
```

Restarted the Docker service to apply changes

```
sudo systemctl restart docker
```

10\. Copied udev rules from elsabot_robot repo (be sure to copy jetson specific rules)

11\. Added ebotagx user to audio group (and then logged out/in).

12\. Changed i2c bus number (from 20 to 9) in device_env_vars.env of elsabot_docker package.

13\. Set ethernet address to 192.168.2.100/255.255.255.0 gw 192.168.2.100 (for teensy uC connection via ethernet).

14\. Enabled bluetooth and paired game controller (press circle button and B button on luna to enable pairing).

15\. Elsabot Jetson Support package changes

a. Revised run_primary_llm.sh

Updated vllm command to use current gemma4 26B model and vllm container.

b. Revised building of jp_docker image (which is used to run TTS/STT).

16\. Cloned and built elsabot packages per elsabot_docker package readme instructions.

(Cloning actually done above before udev rules copied.)

17\. Revised ebotagx user options to login automatically.

18\. Installed chromium (probably did this earlier).

19\. Set AGX power mode to 50W (via dropdown on ubuntu desktop).

20\. Disabled update notifier, see https://askubuntu.com/questions/218755/how-to-disable-the-update-manager-popup

Edited the config file that runs the update-manager

``` 
nano /etc/apt/apt.conf.d/99update-notifier
```

Add '#' infront of the line making it something similar to:

```
#DPkg::Post-Invoke {"if [ -d /var/lib/update-notifier ]; then touch /var/lib/update-notifier/dpkg-run-stamp; fi; if [ -e /var/lib/update-notifier/updates-available ]; then echo > /var/lib/update-notifier/updates-available; fi "; };
```

