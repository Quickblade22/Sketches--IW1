# Sketches-IW1

This repository implements a planning algorithm for the Arcade Learning Environment (ALE) based on the base structure from [rollout-iw](https://github.com/bonetblai/rollout-iw/tree/master).

## Overview

The project integrates sketch-based planning into a simulation planner to solve ALE tasks. Key components are organized as follows:

- **Base Structure**: Adapted from the rollout-iw repository, providing the foundational planning framework.
- **Sketch Implementation**: Located in `Sim_planner`, handling high-level task decomposition and planning.
- **Screen Features**: Defined in `screen.h`, managing visual state processing and feature extraction.
- **Algorithm Core**: Contains the core planning and search algorithms in their respective files.

## Requirements

- **Arcade Learning Environment (ALE)**: Available at [https://github.com/Farama-Foundation/Arcade-Learning-Environment.git](https://github.com/Farama-Foundation/Arcade-Learning-Environment.git)
- **CMake** (latest version)

## How to Operate

1. **Set up ALE**: Download and install the Arcade Learning Environment following its documentation.
2. **Clone and configure**: 
   - Clone this repository.
   - Update the Makefile to point to your ALE installation paths.
3. **Build**:
   - Navigate to the `src` directory.
   - Run `make` to build the `rom_planner` executable.
4. **Run**:
   - Execute `./rom_planner` to see available parameters and usage instructions.
   - Run with appropriate arguments to start planning for a specific ROM.

## Notes

- Ensure all dependencies are correctly linked during compilation.
- Refer to the source code and comments for detailed algorithm descriptions and customization options.
- another repo containting flexible patch sizes and other features are available at https://github.com/Quickblade22/Sketches--IW1_temporary
- 
