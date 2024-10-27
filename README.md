This repository contains a custom-designed GPIO driver for the STM32 Nucleo G474RE microcontroller board. Developed from scratch, this driver enables efficient and flexible GPIO configuration and management, specifically tailored for the STM32G4 series.

Features:
Pin Mode Configuration: Supports input, output, analog, and alternate function modes.
Output Type and Speed: Configurable for push-pull/open-drain and low/medium/high speed.
Pull-up/Pull-down Control: Selectable pull-up, pull-down, or no pull.
Interrupt Handling: Enables GPIO-based interrupts with EXTI support.
Port-Level Control: Allows configuration and control across multiple GPIO ports.
Code Optimization and Readability: Lightweight, modular, and easily integrable with other STM32 peripherals.

Getting Started:
Clone the repository and add it to your STM32 project workspace.
Configure pins using intuitive function calls within your application code.
Build and upload to your STM32 Nucleo G474RE board to see the driver in action.
This driver is designed for embedded systems developers and students looking to understand GPIO functionalities from the ground up, with clean, well-documented code and modular design principles.
