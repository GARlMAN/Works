#!/bin/bash
echo 0 > /sys/module/snd_hda_intel/parameters/power_save
echo N > /sys/module/snd_hda_intel/parameters/power_save_controller
echo 1 > /sys/devices/system/cpu/intel_pstate/no_turbo
