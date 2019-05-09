************
Installation
************
This section contains information on how to build and install the ur_rtde library.

Dependencies
============
.. image:: ../_static/boost-logo.svg
  :width: 60
  :target: https://www.boost.org/
  :alt: Boost

.. image:: ../_static/pybind11-logo.png
  :width: 100
  :target: https://github.com/pybind/pybind11
  :alt: pybind11

* `Boost <https://www.boost.org/>`_
* `pybind11 <https://github.com/pybind/pybind11>`_

.. note::
   The pybind11 dependency is optional and are only required if ur_rtde are to be used with Python. The ur_rtde library
   depends on Boost for networking and threading.

You can install Boost on Ubuntu using:

.. code-block:: shell

   sudo apt-get install libboost-all-dev

*Optionally* if you want to use ur_rtde with Python install pybind11 using:

.. code-block:: shell

    git clone --branch v2.2.4 https://github.com/pybind/pybind11.git --depth 1
    cd pybind11
    mkdir build
    cd build
    cmake ..
    make
    sudo make install

.. role:: bash(code)
   :language: bash

.. note::
    For Python 2.x support use :bash:`cmake -DPYBIND11_PYTHON_VERSION=2.x ..` default is Python3

Compatible Robots
=================
*  All CB-Series from CB3/CB3.1 software 3.3
*  All e-Series

Compatible Operating Systems
============================
Currently tested on:

*  Ubuntu 16.04 (Xenial Xerus)
*  Ubuntu 18.04 (Bionic Beaver)

.. note::
    The ur_rtde should be compatible with Windows 10. pybind11 supports the following compiler on Windows:
    *Microsoft Visual Studio 2015 Update 3 or newer*.

Build Instructions
==================
.. code-block:: shell

    git clone https://gitlab.com/sdurobotics/ur_rtde.git
    cd ur_rtde
    mkdir build
    cd build
    cmake ..
    make
    sudo make install

.. note::
    If you want Python 2.x support you must use :bash:`cmake -DPYBIND11_PYTHON_VERSION=2.x ..` in this step also, since the
    default interpreter is Python3. If you do not want to use Python at all, please
    use :bash:`cmake -DPYTHON_BINDINGS:BOOL=OFF ..`