# SPDR Bot – Simulated Spider Robot for Isaac Sim 🕷️

![SPDR Bot Render](SpdrBot%20v12.png)

**SPDR Bot** is a 4-legged walking robot designed for simulation, experimentation, and reinforcement learning. This repository includes everything you need to run the robot in **NVIDIA Isaac Sim**, including the URDF description, simulation assets, and a sample control script. You can use this project as a learning tool, a sandbox for training walking algorithms, or a base for your own robotics experiments.

---

## 📁 File Overview

| File/Folder | Description |
|-------------|-------------|
| `spdrbot3_direct_project/` | Contains the Isaac Lab **direct project** used for setting up and training policies. Start here if you’re working with Isaac Lab or building your own pipeline. |
| `spyderbot_minimal_URDF/` | The **URDF model** of SPDR Bot. Use this to import the robot into Isaac Sim for physics-based simulation and control. Includes meshes and configuration files. |
| `SpdrBot v12.png` | A high-resolution render of the robot. Preview the full design before diving into simulation. |
| `spdr.usd` | The robot model exported to **USD format**, ready to load in Isaac Sim’s stage view. Use this to visualize or manipulate the robot directly in simulation. |
| `spdr_stage.usd` | It's a spdr.usd file but with a grund plane. |
| `spyderbot_test.py` | A basic **Python test script** to load the robot, move and control joints. Use this to verify movement or develop custom control algorithms. |
| `README.md` | This file :) |

---

## 🎥 Video Demo

[![Watch the demo on YouTube](https://img.youtube.com/vi/YDzHL2JSCHc/maxresdefault.jpg)](https://www.youtube.com/watch?v=YDzHL2JSCHc)

---

## 🛒 Build the Real SPDR Bot

Want to go beyond simulation and build the physical robot?  
You can get all the 3D-printable files, Fusion 360 design, and bonus resources here:  
👉 **[Download SPDR Bot on indystry.cc](https://indystry.cc/product/3d-printable-4-legged-spider-robot/)**

---

## 🚀 Quick Start Tips

- Use the URDF folder to import SPDR Bot into Isaac Sim  
- Open `spdr_stage.usd` for a preconfigured scene  
- Use `spyderbot_test.py` to control joints and test basic behaviors  
- Customize the robot by editing the URDF or using the included Fusion 360 files (available in the paid version)

---

## 🧠 Licensing & Attribution

This project is open for personal and educational use. If you publish work based on it, consider linking back to this repo or [indystry.cc](https://indystry.cc).


