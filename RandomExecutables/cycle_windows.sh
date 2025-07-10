#!/bin/bash

# File to store last focused window id
STATE_FILE="/tmp/cycle_windows_last"

# Get active window id (decimal)
active_win_dec=$(xdotool getactivewindow)
active_win_hex=$(printf '0x%08x\n' $active_win_dec)

# Get PID of active window
pid=$(xdotool getwindowpid $active_win_dec)

# Get all windows with the same PID, in wmctrl order
mapfile -t windows < <(wmctrl -lp | awk -v pid="$pid" '$3==pid {print $1}')

# If only one or no window, just exit
if [ ${#windows[@]} -le 1 ]; then
  exit 0
fi

# Get last window from state file, if it exists
last_win_hex=$(cat "$STATE_FILE" 2>/dev/null)

# Find index of last window or active window in windows array
index=-1
if [[ -n "$last_win_hex" ]]; then
  for i in "${!windows[@]}"; do
    if [ "${windows[$i]}" = "$last_win_hex" ]; then
      index=$i
      break
    fi
  done
fi

# If last window not found, use active window index
if [ $index -eq -1 ]; then
  for i in "${!windows[@]}"; do
    if [ "${windows[$i]}" = "$active_win_hex" ]; then
      index=$i
      break
    fi
  done
fi

# Calculate next index (cycle)
next_index=$(( (index + 1) % ${#windows[@]} ))

# Switch focus to next window
wmctrl -ia "${windows[$next_index]}"

# Save the switched window id for next time
echo "${windows[$next_index]}" > "$STATE_FILE"

# Wait a bit for focus to update
sleep 0.1

