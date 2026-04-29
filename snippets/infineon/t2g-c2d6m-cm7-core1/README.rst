.. _t2g-c2d6m-cm7-core1:

t2g-c2d6m-cm7-core1
###################

Overview
********

This snippet targets Cortex-M7 Core 1 of the Infineon CYT4DN SoC on the
KIT_T2G_C2D6M_LITE board. By default, the ``m7`` board variant builds
for Core 0; applying this snippet redirects the build to Core 1 by
selecting the M7_1.

Usage
*****

To build an application for Cortex-M7 Core 1:

.. code-block:: shell

   west build -b kit_t2g_c2d6m_lite/cyt4dnjbzs/m7 -S t2g-c2d6m-cm7-core1 samples/hello_world
