#!/bin/sh

while [ 1 ]; do

let counter=$counter+1
echo test counter: $counter
iozone -A -r 64K -s 8G -f /mnt/usbdev3/hhh
sleep 2

done

