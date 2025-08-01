Example for Keyword Spotting on Nuvoton's Ethos-U NPU platform
##############################################################

Overview
********

This example shows one Keyword Spotting inference application,
which recognizes the presence of a keyword in a recording,
on Nuvoton's Ethos-U NPU capable platform.
Its very source comes from
Arm's `ML embedded evaluation kit/Keyword Spotting`_ example,
which has more information on the Keyword Spotting application.

.. _ML embedded evaluation kit/Keyword Spotting: https://gitlab.arm.com/artificial-intelligence/ethos-u/ml-embedded-evaluation-kit/-/blob/main/docs/use_cases/kws.md

This example mainly consists of:

- Record audio clip through *DMIC (Digital Microphone)*
- Run Keyword Spotting inference using the `TensorFlow Lite Micro`_ framework
  and the Ethos-U NPU
- Optionally, output the inference result over MQTT

This example runs a model, either `DS-CNN Medium INT8`_ or `MicroNet Medium INT8`_ [#]_,
that has been downloaded from the `Arm model zoo`_.
This model has then been optimized using the `Vela compiler`_ [#]_.

.. [#] Check out `choosing keyword spotting model`_
.. [#] Check out `Optimizing model using Vela`_

.. _TensorFlow Lite Micro: https://github.com/tensorflow/tflite-micro
.. _Arm Model Zoo: https://github.com/ARM-software/ML-zoo
.. _DS-CNN Medium INT8: https://github.com/ARM-software/ML-zoo/tree/master/models/keyword_spotting/ds_cnn_medium/model_package_tf/model_archive/TFLite/tflite_int8
.. _MicroNet Medium INT8: https://github.com/ARM-software/ML-zoo/tree/master/models/keyword_spotting/micronet_medium/tflite_int8


Vela takes a ``tflite`` file as input and produces another ``tflite`` file as output,
where the operators supported by Ethos-U have been replaced by an Ethos-U custom operator.
In an ideal case the complete network would be replaced by a single Ethos-U custom operator.

Support targets
===============

+--------------------+------------------+---------------------+--------------------------+
| Board              | Zephyr target    |NPU                  | Connectivity             |
+====================+==================+=====================+==========================+
| `NuMaker-M55M1`_   | `numaker_m55m1`_ |Ethos-U55 256 MAC    | WiFi ESP8266/Ethernet    |
+--------------------+------------------+---------------------+--------------------------+

.. _NuMaker-M55M1: https://docs.zephyrproject.org/latest/boards/nuvoton/numaker_m55m1/doc/index.html
.. _numaker_m55m1: `NuMaker-M55M1`_
.. _NuMaker-M55M1 board: `NuMaker-M55M1`_

Hardware requirements
=====================

- `NuMaker-M55M1 board`_

.. hint:: This example needs to build and run on Ethos-U NPU capable platform.
   In this document, `NuMaker-M55M1 board`_ is taken for demo.

- Internet connection over WiFi or Ethernet (optional)

Software requirements
=====================

- Host operating system: Windows 10 64-bit or afterwards

.. note:: Most users of Nuvoton's Cortex-M series SoC develop on Windows,
   so this document favors this environment.

.. hint:: The command lines in this document are verified on Windows Git Bash environment.
   For other shell environments, check on how differently shells use line continuation,
   quotation marks, and escapes characters.
   For Bash, line continuation mark is "\\".


- `Zephyr development environment`_

.. _Zephyr development environment: https://docs.zephyrproject.org/latest/develop/index.html

- `Git`_

.. note:: This document favors Git Bash as CLI environment.

.. _Git: https://git-scm.com/

- `Arm GNU Toolchain`_

.. _Arm GNU Toolchain: https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain
.. _Cross GCC compiler: `Arm GNU Toolchain`_

- `OpenNuvoton pyOCD`_

.. note:: The PyPI pyOCD support for M55M1 hasn't been ready.
   Install `OpenNuvoton pyOCD`_ instead:

    .. code-block:: console

        $ pip uninstall pyocd
        $ git clone https://github.com/OpenNuvoton/pyOCD
        $ cd pyOCD
        $ pip install .

    Confirm pyOCD version is ``0.36`` or afterwards:

    .. code-block:: console

        $ pyocd --version
        0.36.1.dev3

.. _OpenNuvoton pyOCD: https://github.com/OpenNuvoton/pyOCD

- `Vela compiler`_ (`PyPI package`__) (optional)

.. hint:: Use for `Optimizing model using Vela`_.

.. _Vela compiler: https://gitlab.arm.com/artificial-intelligence/ethos-u/ethos-u-vela
.. _Vela PyPI package: https://pypi.org/project/ethos-u-vela/
.. __: `Vela PyPI package`_

Building and Running
********************

Building the example
====================

This example doesn't upstream to Zephyr mainline, and exits as `Zephyr workspace application`_.
Assuming that the `zephyrproject` West workspace and the `zephyrproject/zephyr`
repository have settled via `above`__, clone this example:

.. _Zephyr workspace application: https://docs.zephyrproject.org/latest/develop/application/index.html#zephyr-workspace-application

__ `Zephyr development environment`_

.. code-block:: console

    $ cd zephyrproject
    $ mkdir applications
    $ cd applications
    $ git clone https://github.com/OpenNuvoton/NuMaker-Zephyr-TFLM-KWS
    $ cd ..
    
Now, we get back to `zephyrproject`.
Add the `tflite-micro` module to your West manifest and pull it:

.. code-block:: console

    $ west config manifest.project-filter -- +tflite-micro
    $ west update

Dependent on networking options, we have:

- Build the example with WiFi ESP8266 enabled:

.. code-block:: console

    $ west -v build \
    -b numaker_m55m1 \
    applications/NuMaker-Zephyr-TFLM-KWS \
    -- \
    -DEXTRA_CONF_FILE="overlay-wifi.conf" \
    -DSHIELD=esp_8266

- Build the example with Ethernet enabled:

.. code-block:: console

    $ west -v build \
    -b numaker_m55m1 \
    applications/NuMaker-Zephyr-TFLM-KWS \
    -- \
    -DEXTRA_CONF_FILE="overlay-eth.conf"

- Build the example without networking enabled:

.. code-block:: console

    $ west -v build \
    -b numaker_m55m1 \
    applications/NuMaker-Zephyr-TFLM-KWS

Flash the generated image:

.. code-block:: console

    $ west flash

Monitoring the example
======================

To monitor the example, we need to:

- Configure host terminal program with **115200/8-N-1**

- And optionally subscribe MQTT topic if networking is enabled

.. note:: In this example, we connect to MQTT server `test.mosquitto.org`_ [#]_
   with one MQTT client program e.g. browser-based `MQTTX Web`_::

        Host: test.mosquitto.org
        Port: 8081
        Client ID: Auto-generated
        Path: mqtt
        Username/Password: Left blank
        Use SSL: Y (MQTTX Web supports only WSS, no WS)
        Use Websockets: Y (MQTTX Web supports only WSS, no WS)

.. [#] Defined at ``src/MQTT/include/ezmqtt/config.h``.

.. _test.mosquitto.org: https://test.mosquitto.org/
.. _MQTTX Web: https://mqttx.app/web

After running the example via ``west flash``, on host terminal,
you should see messages like:

.. code-block:: console

    <inf> wifi_esp_at: Waiting for interface to come up
    <inf> wifi_esp_at: AT version: 1.7.0.0(Aug 16 2018 00:57:04)
    <inf> wifi_esp_at: SDK version: 3.0.0(d49923c)
    <inf> wifi_esp_at: ESP Wi-Fi ready
    *** Booting Zephyr OS build v4.1.0-5200-g7947930d1602 ***
    <inf> app_kws: BoardInit: complete

    <inf> app_kws: Target system: M55M1

If networking is WiFi ESP8266, you need to connect to WiFi AP first
by running command `wifi connect`.

.. code-block:: console

    <wrn> ezmqtt: Press 'wifi connect' to connect to WiFi AP
    <inf> net_samples_common: Waiting for network...

For example,

.. code-block:: console

    uart:~$ wifi connect -s <SSID> -k 1 -p <PASSPHRASE>
    Connection requested
    Connected

.. important::

    -s  SSID
    -k  Key Management type, 1 for WPA2-PSK
    -p  Passphrase

If networking is enabled, this example tries to connect to MQTT server:

.. code-block:: console

    <inf> net_samples_common: Network connectivity established and IP address assigned
    <inf> ezmqtt: Resovlving address test.mosquitto.org
    <inf> ezmqtt: DNS resolving finished
    <inf> ezmqtt: Resolved address test.mosquitto.org: 5.196.78.28
    <inf> ezmqtt: attempting to connect 137.135.83.217
    <inf> net_mqtt: Connect completed
    <inf> ezmqtt: MQTT client connected!
    <inf> ezmqtt: try_to_connect: 0 <OK>

Show the inference model's information:

.. code-block:: console

    <inf> app_kws: Allocating tensors
    <inf> app_kws: Model INPUT tensors:
    <inf> app_kws:       tensor type is INT8
    <inf> app_kws:       tensor occupies 490 bytes with dimensions
    <inf> app_kws:               0:   1
    <inf> app_kws:               1: 490
    <inf> app_kws: Quant dimension: 0
    <inf> app_kws: Scale[0] = 1.086779
    <inf> app_kws: ZeroPoint[0] = 99
    <inf> app_kws: Model OUTPUT tensors:
    <inf> app_kws:       tensor type is INT8
    <inf> app_kws:       tensor occupies 12 bytes with dimensions
    <inf> app_kws:               0:   1
    <inf> app_kws:               1:  12
    <inf> app_kws: Quant dimension: 0
    <inf> app_kws: Scale[0] = 0.003906
    <inf> app_kws: ZeroPoint[0] = -128
    <inf> app_kws: Activation buffer (a.k.a tensor arena) size used: 69268
    <inf> app_kws: Number of operators: 1
    <inf> app_kws:       Operator 0: ethos-u

We can run the following commands to control kws:

.. code-block:: console

    <wrn> app_kws: Press 'kws next' to resume audio clip inference one-shot
    <wrn> app_kws: Press 'kws resume' to resume audio clip inference continuously
    <wrn> app_kws: Press 'kws suspend' to suspend audio clip inference
    <wrn> app_kws: Press 'kws exit' to exit program

To start audio clip inference, run ``kws resume``:

.. code-block:: console

    uart:~$ kws resume

If networking is enabled, this example shows MQTT information:

.. code-block:: console

    <wrn> app_kws: Subscribe to MQTT topic for inference result:
    <wrn> app_kws: MQTT server: test.mosquitto.org
    <wrn> app_kws: MQTT topic: 764009100000000000000003/kws

If networking is enabled, on MQTT client program, subscribe to topic
named ``<CLIENT_ID>/kws``.

Near the target board, speak out "go go go", and you may see message like:

.. code-block:: console

    <inf> app_kws: For timestamp: 0.000000 (inference #: 0); label: go, score: 0.789062; threshold: 0.750000

.. hint:: Check out ``src/Model/Labels.cpp`` for recognized keywords.

If networking is enabled, on MQTT client program, you would also see above message.

Further reading
***************

Optimizing model using Vela
===========================

This section instructs how to optimize download model using Vela.
We take `DS-CNN Medium INT8`_ as example model to optimize using Vela
and M55M1 Ethos-U NPU as target for which to optimize:

1. Download the `DS-CNN Medium INT8`_ model ``ds_cnn_m_quantized.tflite``
   and place in the directory ``keyword_spotting_ds_cnn_medium_int8``.

2. Optimize the model ``ds_cnn_m_quantized.tflite`` using Vela compiler.
   And we get optimized model ``ds_cnn_m_quantized_vela.tflite``. 

.. code-block:: console

    $ cd keyword_spotting_ds_cnn_medium_int8
    $ vela ds_cnn_m_quantized.tflite \
    --output-dir . \
    --accelerator-config ethos-u55-256

.. important:: M55M1 Ethos-U NPU is Ethos-U55, 256 macs_per_cycle.
   The config value `ethos-u55-256` must match.

3. Convert ``ds_cnn_m_quantized_vela.tflite`` to C array file ``ds_cnn_m_quantized_vela.tflite.h``.

.. code-block:: console

    $ xxd -c 16 -i \
    ds_cnn_m_quantized_vela.tflite \
    ds_cnn_m_quantized_vela.tflite.h

4. Update array content from ``ds_cnn_m_quantized_vela.tflite.h``
   to this example's ``src/Model/ds_cnn_m_quantized_vela_H256.tflite.cpp``.

More build options
===================

This section lists more build options.

This example supports the following choices for keyword spotting input:

- ``CONFIG_NVT_ML_KWS_INPUT_DMIC``: DMIC as KWS input (default)

- ``CONFIG_NVT_ML_KWS_INPUT_WAVE_BLOB``: Wave blob as KWS input

.. code-block:: console

    $ west -v build \
    -b numaker_m55m1 \
    applications/NuMaker-Zephyr-TFLM-KWS \
    -- \
    -DCONFIG_NVT_ML_KWS_INPUT_WAVE_BLOB=y

This example supports the following choices for keyword spotting model:

.. _choosing keyword spotting model:

- ``CONFIG_NVT_ML_KWS_MODEL_DS_CNN``: `DS CNN keyword spotting model`__ (default)

- ``CONFIG_NVT_ML_KWS_MODEL_MICRONET``: `MicroNet keyword spotting model`__

.. __: `DS-CNN Medium INT8`_
.. __: `MicroNet Medium INT8`_

.. code-block:: console

    $ west -v build \
    -b numaker_m55m1 \
    applications/NuMaker-Zephyr-TFLM-KWS \
    -- \
    -DCONFIG_NVT_ML_KWS_MODEL_MICRONET=y

To measure Ethos-U performance, you can enable ``CONFIG_NVT_ML_ETHOS_U_PROFILE``:

.. code-block:: console

    $ west -v build \
    -b numaker_m55m1 \
    applications/NuMaker-Zephyr-TFLM-KWS \
    -- \
    -DCONFIG_NVT_ML_ETHOS_U_PROFILE=y

