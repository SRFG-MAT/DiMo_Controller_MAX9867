# Light Sensor Release Notes

This library provides functions to support use of the AK4954A audio codec found on the CY8CKIT-028-TFT shield.

REMARK (2022 / DiMo Project):
(a) THIS DEMO-CODE (for the audio-chip "AK4954A") is originally based on the following similar "PSoC 62"-based example:
"CY8CKIT-062-BLE" --> Examples --> Peripherals --> "I2S Audio" see also: https://github.com/Infineon/mtb-example-psoc6-i2s/blob/master/README.md

(b) However, in this code-example the underlying low-level libs I2S/I2C have been ported, so that the code is now using the *PDL*-based libraries (=configured by Modus-Toolbox GUI-"Device Configurator")
instead of the I2S *HAL*-library used in the original example (less recommended when targeting easier portable code for development-projects)

GENERAL WARNING: Please *avoid* using/sharing the same HW-PIN-resources (here I2C-Master + I2S) in a mixed-setting with other *HAL*-based code-examples, this would collide (only share the same HW-ressources with PDL-based code libraries)

IMPORTANT Remark: Currently the code only works when audio-chip is correctly connected, this proof-of-concept demo-code doesn't have perfect error-handling (see "//xxxx TODO" below)


https://www.akm.com/content/dam/documents/products/audio/audio-codec/ak4954aen/ak4954aen-en-datasheet.pdf

### What's Included?
* APIs for initializing/de-initializing the driver
* APIs for configuring the audio codec
* APIs for controlling audio data flow

### What Changed?
#### v1.0.1
* Minor updates to avoid warnings on some toolchains
* Minor documentation updates
#### v1.0.0
* Initial release

### Supported Software and Tools
This version of the audio codec library was validated for compatibility with the following Software and Tools:

| Software and Tools                        | Version |
| :---                                      | :----:  |
| ModusToolbox Software Environment         | 2.1     |
| GCC Compiler                              | 9.2     |
| IAR Compiler                              | 8.4     |
| ARM Compiler 6                            | 6.11    |

Minimum required ModusToolbox Software Environment: v2.0

### More information

* [API Reference Guide](https://cypresssemiconductorco.github.io/audio-codec-ak4954a/html/index.html)
* [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)
* [Cypress Semiconductor GitHub](https://github.com/cypresssemiconductorco)
* [ModusToolbox](https://www.cypress.com/products/modustoolbox-software-environment)
* [PSoC 6 Code Examples using ModusToolbox IDE](https://github.com/cypresssemiconductorco/Code-Examples-for-ModusToolbox-Software)
* [PSoC 6 Middleware](https://github.com/cypresssemiconductorco/psoc6-middleware)
* [PSoC 6 Resources - KBA223067](https://community.cypress.com/docs/DOC-14644)

---
© Cypress Semiconductor Corporation, 2019-2020.
