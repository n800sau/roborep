setenv serverip 192.168.1.50
tftp 0x80000000 /boot/demo.bin
tftp 0x80000000 /boot/download.bin

?
tftpboot 0x80000000 /boot/download.bin
tftpboot 0x80000000 /boot/demo.bin

?
go 0x80000000

iminfo 0x80000000


setenv serverip 192.168.1.50 ; tftpboot 0x80000000 /boot/download.bin ; go 0x80000000


bootp 0x80000000 192.168.1.50:/boot/download.bin ; go 0x80000000
bootp 0x80000000 192.168.1.50:/boot/demo.bin ; go 0x80000000
bootp 0x80000000 192.168.1.50:/boot/rtcClock.bin ; go 0x80000000
bootp 0x80000000 192.168.1.50:/boot/gpioLedBlink.bin ; go 0x80000000
bootp 0x80000000 192.168.1.50:/boot/enetEcho.bin ; go 0x80000000
bootp 0x80000000 192.168.1.50:/boot/uartEcho.bin ; go 0x80000000
bootp 0x80000000 192.168.1.50:/boot/usb_dev_serial.bin ; go 0x80000000

bootp 0x82000000 192.168.1.50:/boot/rtcClock.bin ; go 0x82000000


# rename file u-boot
ext4load mmc 0 0x80000000 /boot/uEnv.txt.bak
# it loads file to memory and shows it's size
# e.g.
# 2273 bytes read in 41 ms (53.7 KiB/s)
# to write file back under different name
ext4write mmc 0 0x80000000 /boot/uEnv.txt <file size> 0
# e.g.
ext4write mmc 0 0x80000000 /boot/uEnv.txt 2273 0

