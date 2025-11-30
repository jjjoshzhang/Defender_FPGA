FPGA Space Shooter Game (Defender-Style)
Real-Time 2D Arcade Game on the DE1-SoC FPGA (Verilog, VGA, PS/2)

This project implements a complete real-time 2D space shooter game on the DE1-SoC FPGA, built entirely in Verilog.
It features smooth VGA graphics, PS/2 keyboard controls, hardware-accelerated game logic, sprite rendering, and win/lose result screens.

🎮 Features

Real-time gameplay at 50 MHz

640×480 VGA output with 9-bit color

PS/2 keyboard controls (W/A/S/D + M to fire)

Modular hardware game engine with reusable Verilog components

Sprite rendering using .mif files

Enemy movement, missile firing, and collision detection

Random alien vertical spawn using an LFSR

Score tracking with HEX display and game-end conditions

Menu screen, clear screen, and result (win/lose) screens

Priority-based drawing: missile > spacecraft > alien

🕹️ Gameplay Overview

Player Controls

Move: W / A / S / D

Fire missile: M

Alien Behavior

Spawns at random Y positions

Moves from right → left

Player loses if alien reaches left boundary

Win Condition

Score 10 hits → Win screen (green)

Lose Condition

Alien reaches x = 0 → Lose screen (red)
