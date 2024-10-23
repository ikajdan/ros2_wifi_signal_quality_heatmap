#!/bin/bash

export LC_ALL=C

INTERFACE=$1

if [ -d /host_proc ]; then
    PROC_DIR=/host_proc
else
    PROC_DIR=/proc
fi

proc="$(grep "${INTERFACE}" "${PROC_DIR}/net/wireless")"
iw="$(iw dev "${INTERFACE}" link)"

link_quality="$(echo "${proc}" | awk '{print $3}')"
signal_level="$(echo "${proc}" | awk '{print $4}')"
noise_level="$(echo "${proc}" | awk '{print $5}')"

link_quality="${link_quality%.}"
signal_level="${signal_level%.}"
noise_level="${noise_level%.}"

ssid=$(echo "${iw}" | grep 'SSID:' | awk '{print $2}')
freq=$(echo "${iw}" | grep 'freq:' | awk '{print $2}')
rx_bitrate=$(echo "${iw}" | grep 'rx bitrate:' | awk '{print $3}')
tx_bitrate=$(echo "${iw}" | grep 'tx bitrate:' | awk '{print $3}')

echo "SSID: $ssid"
echo "Frequency: $freq"
echo "Link Quality: $link_quality"
echo "Signal Level: $signal_level"
echo "Noise Level: $noise_level"
echo "RX Bitrate: $rx_bitrate"
echo "TX Bitrate: $tx_bitrate"
