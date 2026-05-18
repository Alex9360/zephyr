.. zephyr:board:: kit_t2g_b_e_lite

Overview
********

The KIT_T2G-B-E_LITE kit enables you to evaluate and develop applications using the TRAVEO™ T2G Body
Entry family CYT2BL device. The TRAVEO™ T2G B-E MCU is specifically designed for automotive body
applications and is a true programmable embedded system-on-chip, integrating a 160-MHz Arm® Cortex®-M4F
CPU as the primary application processor and a 100-MHz Arm® Cortex®-M0+ that supports the following:

- Low-power operations (Active, Sleep, Low-power Sleep, Deep Sleep, and Hibernate modes)
- Code flash, work flash, and on-chip SRAM
- CAN FD, LIN, and other automotive peripherals

The TRAVEO™ T2G B-E Lite kit is a 100-pin evaluation board featuring an onboard KitProg3
programmer/debugger, CAN FD transceiver, Shield2Go and mikroBUS connectors, Arduino-compatible headers,
three user LEDs, one potentiometer, and two user push buttons.

Hardware
********

For more information about KIT_T2G-B-E_LITE:

- `kit_t2g_b_e_lite Board Website`_
- `T2G_B_E SoC Website`_

Kit Features
============

- Evaluation board for CYT2BL MCU in 100-pin package, Arm® Cortex®-M4F CPU at 160-MHz and Arm® Cortex®-M0+ CPU at 100-MHz
- Headers compatible with Arduino for interfacing Arduino shields
- Shield2Go and mikroBUS connector interfaces
- Fully compatible with ModusToolbox™
- KitProg3 on-board SWD programmer/debugger, USB-UART, and USB-I2C bridge functionality through USB connector
- CAN FD transceiver, three user LEDs, one potentiometer, and two user push buttons
- Operating voltage from 3.3 V to 5.0 V for CYT2BL

Supported Features
==================

.. zephyr:board-supported-hw::

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Building
========

Build the :zephyr:code-sample:`blinky` sample application for the Cortex®-M0+ core:

.. zephyr-app-commands::
   :zephyr-app: samples/basic/blinky
   :board: kit_t2g_b_e_lite/cyt2bl5cae/m0p
   :goals: build

To release the Cortex®-M4 core from M0+ during boot, pass the Kconfig option at build time. Only enable
the M4 when valid firmware is present at its flash partition; otherwise the core will fetch invalid
instructions and fault.

.. zephyr-app-commands::
   :zephyr-app: samples/basic/blinky
   :board: kit_t2g_b_e_lite/cyt2bl5cae/m0p
   :gen-args: -DCONFIG_SOC_CYT2BX_START_M4=y
   :goals: build

Build for the Cortex®-M4 core:

.. zephyr-app-commands::
   :zephyr-app: samples/basic/blinky
   :board: kit_t2g_b_e_lite/cyt2bl5cae/m4
   :goals: build

.. note:: Per the CYT2BL start-up sequence, only the Cortex®-M0+ core is released from reset by
   ROM/flash boot. The Cortex®-M0+ application is responsible for enabling the Cortex®-M4 core by
   selecting ``CONFIG_SOC_CYT2BX_START_M4`` at build time. The M0+ startup code programs the M4 vector
   table base, enables power for the M4, and releases it from reset.

Flashing
========

The KIT_T2G-B-E_LITE includes an onboard programmer/debugger (`KitProg3`_) to provide debugging, flash
programming, and serial communication over USB. Flash and debug commands use OpenOCD and require a
custom Infineon OpenOCD version that supports KitProg3.

Both the full `ModusToolbox`_ and the `ModusToolbox Programming Tools`_ packages include Infineon
OpenOCD. Installing either of these packages will also install Infineon OpenOCD. If neither package is
installed, a minimal installation can be done by downloading the `Infineon OpenOCD`_ release for your
system and manually extracting the files to a location of your choice.

.. note:: Linux requires device access rights to be set up for KitProg3. This is handled automatically
   by the ModusToolbox and ModusToolbox Programming Tools installations. For a minimal installation,
   execute ``openocd/udev_rules/install_rules.sh``.

The path to the installed Infineon OpenOCD executable must be available to the ``west`` tool. The
examples below use a permanent CMake argument to set the CMake variable ``OPENOCD``.

Set the OpenOCD path once:

.. code-block:: shell

   west config build.cmake-args -- -DOPENOCD=path/to/infineon/openocd/bin/openocd

Build and flash for the Cortex®-M0+ core:

.. code-block:: shell

   west build -b kit_t2g_b_e_lite/cyt2bl5cae/m0p -p always samples/basic/blinky
   west flash
   west debug

Build and flash for the Cortex®-M0+ core with the M4 core released during boot:

.. code-block:: shell

   west build -b kit_t2g_b_e_lite/cyt2bl5cae/m0p -p always samples/basic/blinky \
       -- -DCONFIG_SOC_CYT2BX_START_M4=y
   west flash
   west debug

Build and flash for the Cortex®-M4 core:

.. code-block:: shell

   west build -b kit_t2g_b_e_lite/cyt2bl5cae/m4 -p always samples/basic/blinky
   west flash
   west debug

Once the gdb console starts after executing the west debug command, you may set breakpoints and perform
standard GDB debugging.

References
**********

.. target-notes::

.. _T2G_B_E SoC Website:
   https://www.infineon.com/products/microcontroller/32-bit-traveo-t2g-arm-cortex/for-body/t2g-cyt2b7

.. _kit_t2g_b_e_lite Board Website:
   https://www.infineon.com/evaluation-board/KIT-T2G-B-E-LITE

.. _ModusToolbox:
   https://softwaretools.infineon.com/tools/com.ifx.tb.tool.modustoolbox

.. _ModusToolbox Programming Tools:
   https://softwaretools.infineon.com/tools/com.ifx.tb.tool.modustoolboxprogtools

.. _Infineon OpenOCD:
   https://github.com/Infineon/openocd/releases/latest

.. _KitProg3:
   https://github.com/Infineon/KitProg3
