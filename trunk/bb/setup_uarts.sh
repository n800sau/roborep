#!/bin/sh

#5v tx 3.3k -(3.26v)- 6.2k grn
#3v -|<- 5v

echo 0 > /sys/kernel/debug/omap_mux/uart1_txd
echo 20 > /sys/kernel/debug/omap_mux/uart1_rxd

echo "UART 1 (TX):"
cat /sys/kernel/debug/omap_mux/uart1_txd
echo

echo "UART 1 (RX):"
cat /sys/kernel/debug/omap_mux/uart1_rxd
echo

echo 1 > /sys/kernel/debug/omap_mux/spi0_d0
echo 21 > /sys/kernel/debug/omap_mux/spi0_sclk

echo "UART 2 (TX):"
cat /sys/kernel/debug/omap_mux/spi0_d0
echo

echo "UART 2 (RX):"
cat /sys/kernel/debug/omap_mux/spi0_sclk
echo


echo 6 > /sys/kernel/debug/omap_mux/gpmc_wpn
echo 26 > /sys/kernel/debug/omap_mux/gpmc_wait0

echo "UART 4 (TX):"
cat /sys/kernel/debug/omap_mux/gpmc_wpn
echo

echo "UART 4 (RX):"
cat /sys/kernel/debug/omap_mux/gpmc_wait0
echo
