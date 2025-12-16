sudo ip link add dev vcan0 type vcan

sudo ip link set up vcan0

ip link show vcan0


candump vcan0


cansend vcan0 123#DEADBEEF
