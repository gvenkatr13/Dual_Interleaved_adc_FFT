# STM32H7H7 Dual Interleaved ADC FFT

## Overview

This project utilizes the STM32H7H7 microcontroller to perform Fast Fourier Transform (FFT) computations on ADC samples acquired in Dual Interleaved Mode. It leverages Direct Memory Access (DMA) for efficient data transfer between the ADC and memory, reducing CPU overhead.

## Features

- **Microcontroller**: STM32H747XIH6
- **ADC Configuration**: Dual Interleaved Mode with DMA
- **FFT Library**: CMSIS-DSP

## Implementation

The project is implemented in C/C++ using STM32 CubeIDE for development.

**ADC Configuration**:
   - Configures the ADC in Dual Interleaved Mode to acquire samples from multiple channels simultaneously on pin PA_4.


