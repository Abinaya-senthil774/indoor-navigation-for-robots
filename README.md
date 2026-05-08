# Hierarchical Optimization Technique for Placement of Visual Landmarks for Robot Navigation in Repetitive Structured Indoor Environments

# Autonomous Indoor Navigation System Using Visual Beacons

## Description
This repository contains the source code, simulations, and documentation for a cost-effective, vision-based autonomous indoor navigation system designed for mobile robots. Navigating repetitive, GPS-denied environments—such as hotel corridors or hospitals—often causes spatial ambiguity and traditional SLAM failures. To resolve this without relying on expensive LiDAR systems, this project utilizes passive ArUco markers as navigation beacons. 

The core of this repository is a novel three-stage hierarchical optimization algorithm. Rather than uniformly distributing tags (which causes visual clutter and infrastructure redundancy), the algorithm mathematically calculates the optimal placement of Functional, Junction, and Corridor tags. The system successfully maximizes localization availability while drastically reducing the required infrastructure, and has been validated across MATLAB, Webots, and real-world hardware.

---
## Repository Structure
```text
autonomous-indoor-navigation/
├── docs/                    # Documentation, project reports, and presentations
│
├── src/                     # Source code for optimization and deployment
│   ├── algorithm/           # MATLAB/Python scripts for the 3-stage optimization
│   ├── simulation/          # Webots R2025a environments (.wbt) and controllers
│   ├── hardware/            # Deployment scripts for the Yahboom Jetson bot
│   └── manual_baseline/     # Baseline scripts for uniform tag placement testing
│
├── assets/                  # Images for documentation
├── LICENSE
└── README.md
```
---
## Key Features
* **Cost-Effective Localization:** Replaces expensive LiDAR and computationally heavy SLAM with a lightweight, vision-based ArUco marker system.
* **Hierarchical Optimization Algorithm:** Automates marker placement using a greedy search algorithm that balances coverage and redundancy against infrastructure costs.
* **Deterministic Junction Safety:** Mandates dual-tag placement at intersections, achieving a 100% pre-turn success rate and preventing localization loss during sharp maneuvers.
* **Cross-Domain Validation:** Fully tested and validated in mathematical models (MATLAB), high-fidelity digital twin simulations (Webots R2025a), and real-time physical environments.
* **Edge-Hardware Ready:** Designed to run seamlessly on resource-constrained platforms, specifically validated on the NVIDIA Jetson Nano.

---

## Workflow & Methodology
![methodology](/assets/methodology.jpeg)

The tag placement strategy follows a structured, visibility-aware optimization framework:

1. **Environment Definition & Feasibility Analysis:** Extracts corridor geometry and uses camera intrinsics (focal length, field of view) alongside pixel detection thresholds to compute the maximum detection distance ($d_{max}$).
2. **Visibility Matrix Generation:** Generates a candidate pool of tags along wall surfaces and maps them against sampled robot poses to check for distance, bearing angle, and occlusion constraints.
3. **Multi-Stage Tag Placement:**
   * **Stage 1 (Functional Tags):** Prioritizes high-value task areas, placing markers specifically at doorways and functional zones.
   * **Stage 2 (Junction Tags):** Secures error-prone intersections (L- and T-junctions) by placing outer and inner tags to ensure continuous visibility during heading changes.
   * **Stage 3 (Corridor Tags):** Fills remaining straight-line gaps using a greedy iterative algorithm that penalizes redundancy and minimizes blackout regions.
     
![methodology](/src/algorithm/algorithm/algorithm_flowchart.png)
---
## Results
![methodology](/assets/fig10.png)

![methodology](/assets/fig10b.png)

![methodology](/assets/fig11.jpg)

![methodology](/assets/fig12a.png)

The proposed hierarchical optimization technique yielded significant improvements over manual and uniform placement strategies:

* **Localization Availability:** Achieved 94.2% availability in simulated environments and 92% - 98.6% in real-time hardware validation.

* **Infrastructure Reduction:** Reduced total marker infrastructure by 82% (from 178 candidate tags down to 32 optimized tags) compared to dense, uniform deployment.

* **Reduced Blackout:** Minimized maximum blackout lengths to just 0.3 meters in straight corridors.

* **Consistent Visibility:** Maintained a mean visibility of 1.011 tags per frame across the robot's entire trajectory.

* **Reliability:** Hardware validation on the Yahboom Jetson bot (NVIDIA Jetson Nano 4GB) confirmed the algorithm's robustness, matching theoretical predictions even at varying operational speeds


