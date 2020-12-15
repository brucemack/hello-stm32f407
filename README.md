Bring-Up Firmware for STM32F407 Board
=====================================



I2S Notes
---------
There is an eratta in the STM32F407 chip related to I2S in slave mode.  It is important to make sure the
slave device (I2S3 in our case) is enabled first (while the WS line is high) before enabling the master
to avoid a clock synchronization bug.  Please see section 2.6.1 in this document for more information:

https://www.st.com/resource/en/errata_sheet/dm00037591-stm32f405-407xx-and-stm32f415-417xx-device-limitations-stmicroelectronics.pdf

