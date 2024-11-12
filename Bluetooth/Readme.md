# Bluetooth library using MCAL

This library allows the communication between a STM32-Nucleo-F401RE and a HM17 Bluetooth module.

To use the module on USART2, you need to adjust the solder bridges on the bottom of the board.

SB63 and SB62 need to be bridged
SB13 and SB14 need to be removed

This disables the possibility to communicate over the Virtual COM port. You can leave SB13 installed to monitor outgoing messages.
