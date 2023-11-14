# Overview
This repository contains the nRF52832 embedded systems application code for the RepXcel project.

# Setup & Installations
The following installations are required to compile software and program the microcontroller.
- [J-Link Software 7.92](https://www.segger.com/downloads/jlink/JLink_Windows_x86_64.exe)
- [Arm GNU Toolchain 10.3.1](https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-win32.exe?rev=29bb46cfa0434fbda93abb33c1d480e6&hash=3C58D05EA5D32EF127B9E4D13B3244D26188713C)
- [nRF Command Line Tools 10.23.2](https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/desktop-software/nrf-command-line-tools/sw/versions-10-x-x/10-23-2/nrf-command-line-tools-10.23.2-x64.exe)
- [Make for Windows 3.81](https://gnuwin32.sourceforge.net/downlinks/make.php)

# Debug
The following debug configuration can be used with the [Cortex-Debug](vscode:extension/marus25.cortex-debug) extension on Visual Studio Code.

```json
{
    "name": "Jlink Lite Cortex-M Debug NRF52832",
    "cwd": "${fileDirname}",
    "executable": "./d52/blank/armgcc/_build/nrf52832_xxaa.out",
    "request": "launch",
    "type": "cortex-debug",
    "runToEntryPoint": "main",
    "servertype": "jlink",
    "serverpath": "C:/Program Files/SEGGER/JLink/JLinkGDBServerCL.exe",
    "armToolchainPath": "C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin",
    "device": "NRF52832_xxAA",
    "interface": "swd",
}
```