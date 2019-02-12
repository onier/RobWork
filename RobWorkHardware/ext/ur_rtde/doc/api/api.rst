*************
API Reference
*************

The ur_rtde library API consists of the following parts:

* :ref:`rtde_control_interface.h <rtde-control-api>`: The RTDE control interface
* :ref:`rtde_receive_interface.h <rtde-receive-api>`: The RTDE receive interface
* :ref:`rtde.h <rtde-api>`: The RTDE class
* :ref:`script_client.h <script-client-api>`: Script client
* :ref:`dashboard_client.h <dashboard-client-api>`: Dashboard client

.. _rtde-control-api:

RTDE Control Interface API
==========================

.. doxygenclass:: RTDEControlInterface
   :project: ur_rtde
   :members:

.. _rtde-receive-api:

RTDE Receive Interface API
==========================

.. doxygenclass:: RTDEReceiveInterface
      :project: ur_rtde
      :members:
      :undoc-members:

.. _rtde-api:

RTDE Class API
==============

.. doxygenfile:: rtde.h


.. _script-client-api:

Script Client API
=================

.. doxygenfile:: script_client.h


.. _dashboard-client-api:

Dashboard Client API
====================

.. doxygenfile:: dashboard_client.h