# Overview
This repository contains the nRF52832 embedded systems application code for the RepXcel project.

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