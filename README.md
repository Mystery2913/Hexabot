# Hexabot - A hexapedal robot from scratch
![GitHub release (latest by date)](https://img.shields.io/github/v/release/Mystery2913/Hexabot)

## Introduction
Author: **Dylan Sharm**

**Hexabot** is a fully developed hexapedal robot made using 3D printing. Comprising of a 3D printed body, it contains the leg mechanisms for 3 joints to move each leg in 3 dimensions. All 6 legs allow for a stable base for walking around. The system is controlled by a local website run on a Raspberry Pi 4, while control system features speed control and standing control. The robot is powered by a 7.4v 7500mAh battery that is removable from the robot with power indicator light included.

## Building Hexabot
To build Hexabot, follow the **[user manual](hexabot-user-manual.pdf)**. It details all instructions of how to make Hexabot including components and tools, materials, assembly instructions and diagrams, software instructions, supporting tips, and safety information. Using this, a fully contracted Hexabot can be made.

## Hexabot Software Installation
Install Raspberry Pi OS onto raspberry pi. After installation, enable I2C and SSH in `Preferences > Raspberry Pi Configuration`

Install `git` onto system:
```
sudo apt update
sudo apt install git
```
Verify the installation. Should return lasted version (2.32.0):
```
git --version
```
Then after installation and configuration, navigate to Projects in users:
```
cd Projects
```
Clone repository:
```
git clone https://github.com/Mystery2913/Hexabot.git
```