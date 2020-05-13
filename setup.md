# Building Environment

This is document describes how to build a real-time environment for Raspberry Pi.

The steps in this article will set up the following items.

- RT preempt real-time kernel
- Fixed operation frequency at 1.0 GHz
- Aggregate kernel threads to CPU0
- Exclude CPU1~CPU3 from scheduling
- CPU1~CPU3 tickless
- Disable FIQ

At the time of the test, the following items will be considered

- Set a real-time scheduling policy (e.g.  RR)
- Fix the test process's operating core to one of CPU1~CPU3 (use taskset).

## Target Hardware

- Raspberry Pi 3 Model B+
- SD card (Raspbean installed)
- Broadcom BCM2837B0

## Building RT_PREEMPT Patched kernel

1. Compile real-time kernel

Use RT_PREEMPT patched branch rpi-4.19.y-rt.

   ```shell
   # host PC 
   $ mkdir ~/rpi-kernel && cd ~/rpi-kernel
   $ git clone https://github.com/raspberrypi/linux.git -b rpi-4.19.y-rt # RT_PREEMPT patched
   $ git clone https://github.com/raspberrypi/tools.git --depth 1
   $ cd linux
   $ export ARCH=arm
   $ which arm-linux-gnueabihf-cpp
   /home/username/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-cpp
   # arm-linux-gnueabihf-*** confirm absolute path
   $ export CROSS_COMPILE=/home/username/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-
   $ export KERNEL=kernel7
   $ mkdir ~/rpi-kernel/boot
   $ export INSTALL_MOD_PATH=~/rpi-kernel/boot
   $ export INSTALL_DTBS_PATH=~/rpi-kernel/boot
   $ cd linux
   $ make bcm2709_defconfig # bcm2835_defconfig didn't work.
     HOSTCC  scripts/basic/fixdep
     HOSTCC  scripts/kconfig/conf.o
     YACC    scripts/kconfig/zconf.tab.c
     LEX     scripts/kconfig/zconf.lex.c
     HOSTCC  scripts/kconfig/zconf.tab.o
     HOSTLD  scripts/kconfig/conf
   #
   # configuration written to .config
   #
   $ make menuconfig
   - Select: Preemptive Kernel(Low-Latency Desktop)
    General Setup>
     Preemption Model>
      Preemptive Kernel(Low-Latency Desktop)
   - Enable CONFIG_PREEMPT_RT_FULL
    General setup>
     Timers subsystem>
      High Resolution Timer Support
   - Enable tickless
    General setup>
     Timers subsystem>
      Timer tick handling>
       Full dyntick system (tickless)
   - Set CONFIG_HZ to 1000Hz>
    Kernel Features>
     Timer frequency = 1000 Hz
   - Save to .config
   % vim .config # disable debugs
   $ grep -i debug .config | grep -v ^#
   CONFIG_GENERIC_IRQ_DEBUGFS=n
   CONFIG_SLUB_DEBUG=n
   CONFIG_DEBUG_ALIGN_RODATA=n
   CONFIG_BLK_DEBUG_FS=n
   CONFIG_BT_DEBUGFS=n
   CONFIG_WIMAX_DEBUG_LEVEL=8
   CONFIG_B43LEGACY_DEBUG=n
   CONFIG_RTLWIFI_DEBUG=n
   CONFIG_WIMAX_I2400M_DEBUG_LEVEL=8
   CONFIG_USB_SERIAL_DEBUG=n
   CONFIG_OCFS2_DEBUG_MASKLOG=n
   CONFIG_JFFS2_FS_DEBUG=0
   CONFIG_CIFS_DEBUG=n
   CONFIG_DEBUG_FS=n
   CONFIG_DEBUG_KERNEL=n
   CONFIG_HAVE_DEBUG_KMEMLEAK=n
   CONFIG_ARCH_HAS_DEBUG_VIRTUAL=n
   CONFIG_DEBUG_MEMORY_INIT=n
   CONFIG_SCHED_DEBUG=n
   CONFIG_DEBUG_PREEMPT=n
   CONFIG_LOCK_DEBUGGING_SUPPORT=n
   CONFIG_DEBUG_BUGVERBOSE=n
   CONFIG_DEBUG_LL_INCLUDE="mach/debug-macro.S"
   CONFIG_UNCOMPRESS_INCLUDE="debug/uncompress.h"
   
   $ make -j 10 zImage modules dtbs
   ~~~~
     OBJCOPY arch/arm/boot/zImage
     Kernel: arch/arm/boot/zImage is ready
   $ mkdir -p ~/rpi-kernel/boot/overlays
   $ make modules_install
   $ make dtbs_install
   $ cp arch/arm/boot/zImage ../boot/
   $ rsync -avz ../boot pi@[ip_address_for_raspberry_pi]:~
   ```

   ```shell
   ## raspberry pi
   
   $ sudo cp ~/boot/*.dtb /boot/
   $ sudo cp ~/boot/overlays/*.dtb* /boot/overlays/
   $ sudo cp ~/boot/overlays/README /boot/overlays/
   $ sudo cp ~/boot/zImage /boot/rt-kernel.img
   $ sudo mv ~/boot/lib/modules/* /lib/modules
   $ sudo vim /boot/config.txt
   # add following line to /boot/config.txt
   kernel=rt-kernel.img
   $ sudo vim /boot/cmdline.txt
   # add to the end.
   #  dwc_otg.fiq_fsm_enable=0 dwc_otg.fiq_enable=0 dwc_otg.nak_holdoff=0 dwg_otg.speed=1 rcu_nocbs=0 nohz_full=1-3 isolcpus=1-3
   # disable fiq, fix kenel threads to cpu 0, cpu1~3 tickless
   
   # example
   # console=serial0,115200 console=tty1 root=PARTUUID=266982c5-02 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles dwc_otg.fiq_fsm_enable=0 dwc_otg.fiq_enable=0 dwc_otg.nak_holdoff=0 dwg_otg.speed=1 rcu_nocbs=0 nohz_full=1-3 isolcpus=1-3
   
   $ sudo reboot
   $ uname -a
   Linux raspberrypi 4.19.71-rt24-v7+ #5 SMP PREEMPT RT Thu Feb 13 14:16:03 JST 2020 armv7l GNU/Linux
   # confirm "-rt"
   $ sudo modprobe config
   $ zcat /proc/config.gz > .config
   $ grep -i preempt .config 
   CONFIG_PREEMPT=y # y
   CONFIG_PREEMPT_RT_BASE=y # y
   ~~
   CONFIG_PREEMPT_RT_FULL=y # y
   ~~
   CONFIG_NO_HZ_FULL=y # y
   $ dmesg | grep -i fiq
   [    0.000000] Kernel command line: coherent_pool=1M 8250.nr_uarts=0 bcm2708_fb.fbwidth=1824 bcm2708_fb.fbheight=984 bcm2708_fb.fbswap=1 smsc95xx.macaddr=B8:27:EB:0F:81:5B vc_mem.mem_base=0x3ec00000 vc_mem.mem_size=0x40000000  console=ttyS0,115200 console=tty1 root=PARTUUID=266982c5-02 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles dwc_otg.fiq_fsm_enable=0 dwc_otg.fiq_enable=0 dwc_otg.nak_holdoff=0 dwc_otg.speed=1
   [    0.957023] dwc_otg: FIQ disabled# disabled
   [    0.957041] dwc_otg: FIQ split-transaction FSM disabled
    $ mpstat -P ALL # cpu1~cpu3 load are zero.
   Linux 4.19.55-rt24-v7+ (raspberrypi)    03/13/2020      _armv7l_        (4 CPU)
   
   10:12:38 AM  CPU    %usr   %nice    %sys %iowait    %irq   %soft  %steal  %guest  %gnice   %idle
   10:12:38 AM  all    4.79    0.00    8.82    0.32    0.00    0.05    0.00    0.00    0.00   86.03
   10:12:38 AM    0   21.28    0.01   39.21    1.40    0.00    0.21    0.00    0.00    0.00   37.88
   10:12:38 AM    1    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00  100.00
   10:12:38 AM    2    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00  100.00
   10:12:38 AM    3    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00  100.00
   $ cat /proc/interrupts # Only the number of interrupts handled by CPU0 increase.
              CPU0       CPU1       CPU2       CPU3
    17:         82          0          0          0  ARMCTRL-level   1 Edge      3f00b880.mailbox
    18:         36          0          0          0  ARMCTRL-level   2 Edge      VCHIQ doorbell
    33:     856926          0          0          0  ARMCTRL-level  41 Edge      dwc_otg, dwc_otg_pcd, dwc_otg_hcd:usb1
    40:          0          0          0          0  ARMCTRL-level  48 Edge      bcm2708_fb DMA
    42:        341          0          0          0  ARMCTRL-level  50 Edge      DMA IRQ
    44:       4237          0          0          0  ARMCTRL-level  52 Edge      DMA IRQ
    80:       4197          0          0          0  ARMCTRL-level  88 Edge      mmc0
    81:       7425          0          0          0  ARMCTRL-level  89 Edge      uart-pl011
    86:       3703          0          0          0  ARMCTRL-level  94 Edge      mmc1
   161:          0          0          0          0  bcm2836-timer   0 Edge      arch_timer
   162:     110685        157        142        127  bcm2836-timer   1 Edge      arch_timer
   165:          0          0          0          0  bcm2836-pmu   9 Edge      arm-pmu
   166:          0          0          0          0  lan78xx-irqs  17 Edge      usb-001:004:01
   IPI0:          0          0          0          0  CPU wakeup interrupts
   IPI1:          0          0          0          0  Timer broadcast interrupts
   IPI2:         15         26         26         24  Rescheduling interrupts
   IPI3:          0          5          5          5  Function call interrupts
   IPI4:          0          0          0          0  CPU stop interrupts
   IPI5:         40          5          5          5  IRQ work interrupts
   IPI6:          0          0          0          0  completion interrupts
   Err:          0
   $ cat /proc/softirqs # Only the number of interrupts handled by CPU0 increase.
                       CPU0       CPU1       CPU2       CPU3
             HI:       3120          0          0          0
          TIMER:     111577        156        141        126
         NET_TX:          9          0          0          0
         NET_RX:        462          0          0          0
          BLOCK:          0          0          0          0
       IRQ_POLL:          0          0          0          0
        TASKLET:      16745          0          0          0
          SCHED:          0          0          0          0
        HRTIMER:       1771          0          0          0
            RCU:          0          0          0          0
   
   
   ```
The process of the name like dwc_otg always uses about 30% CPU because FIQ is disabled. Details here.
[RT kernel: Unusual dwc_otg high 8k/s interrupt rate using quite some CPU? - Navio / Edge / Flight stack - Community Forum](https://community.emlid.com/t/rt-kernel-unusual-dwc-otg-high-8k-s-interrupt-rate-using-quite-some-cpu/1002)


If the supply voltage drops below 4.7V, the message "Unde-voltage detected!" will be displayed and the measurement result will change. Pay attention to the power supply.

### How to check constant CPU frequency and absence of performance dropping.

   ```shell
   $ echo '1' | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/stats/reset # reset cpu-stats
   $ stress -c 4 & # CPU load
   pi@raspberrypi:~ $ vcgencmd measure_temp
   temp=53.7C # detecting over 70 degrees enables CPU-throttling.
   $ vcgencmd get_throttled
   throttled=0x0 # 0x0 indicates no performance dropping.
   $ cpufreq-info
   analyzing CPU 0:
     cpufreq stats: 600 MHz:0.00%, 1000 MHz:100.00%  # all core operated at frequency of1000MHz
     ~~~
   ```

** ※ use nohup, tmux or screen command during the experiments **

## Cyclictest

Compare the real time performance before and after the configuration with cyclictest, a linux tool for measurement.


```shell
$ taskset -c 0 stress -c 1 & # impose CPU-stress and IO-stress for each core
$ taskset -c 1 stress -c 1 &
$ taskset -c 2 stress -c 1 &
$ taskset -c 3 stress -c 1 &
$ taskset -c 0 stress -i 1 &
$ taskset -c 1 stress -i 1 &
$ taskset -c 2 stress -i 1 &
$ taskset -c 3 stress -i 1 &
$ ps -eo pid,comm,psr | grep kworker | awk '{print "chrt -r -p 98 "$1}' | sh
# raise kworker priorities to 98
```

```shell
$ taskset -c 0 cyclictest -p 98 -m -t1 -n -D 3h -i $1 -a 1 --policy=rr -h500 -q > result.txt
# run the script at cpu1 with RR priority 98 for 3 hours.
$ ps -em -o pid,tid,policy,pri,ni,rtprio,comm,psr | grep cyclic -A 2
# Check per-thread priorities and scheduling policies
$ grep -v -e "^#" -e "^$"  result.txt | tr " " "," > histogram.csv # Extract only the histogram part
```

![pure_vs_rt](/uploads/fdfbcfc6f52a6fc42b9d30f5228edde2/pure_vs_rt.png)

The figure is a histogram of the latency to nanosleep wake-up, measured with a 200 us period sleep.
The pure kernel result shows that sleep duration reaches from 210us to 300us at worst.
The real-time kernel the result shows that sleep duration reaches from 210 us to 240 us at worst.

In the pure kernel, nanosleep wakes up faster than real-time kernel at the most cases.
When compared at the worst values, the real-time kernel has less jitter and better real-time performance.

## ROS2 Build

The build excludes visualization-related packages and configures cmake-args.


```
$ touch src/ros-visualization/COLCON_IGNORE src/ros2/rviz/COLCON_IGNORE
$ colcon build --symlink-install  --packages-skip-build-finished --continue-on-error --cmake-args "-DCMAKE_SHARED_LINKER_FLAGS='-latomic'" "-DCMAKE_EXE_LINKER_FLAGS='-latomic'"
```
