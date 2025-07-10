 sudo nano /sys/module/snd_hda_intel/parameters/power_save
 sudo nano /sys/module/snd_hda_intel/parameters/power_save_controller
 
 
 echo 1 | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo
 
 
