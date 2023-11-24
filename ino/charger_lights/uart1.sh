echo 0 > /sys/kernel/debug/omap_mux/uart1_txd
echo 20 > /sys/kernel/debug/omap_mux/uart1_rxd

echo "UART 1 (TX):"
cat /sys/kernel/debug/omap_mux/uart1_txd
echo

echo "UART 1 (RX):"
cat /sys/kernel/debug/omap_mux/uart1_rxd
echo
