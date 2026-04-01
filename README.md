# Distributed Swarm Control with Connectivity Maintenance and Obstacle Avoidance

[cite_start]This project implements a decentralized control framework for the safe navigation of a micro-UAV swarm in cluttered 3D environments[cite: 6]. [cite_start]The architecture utilizes **High-Order Control Barrier Functions (HO-CBFs)** embedded within a **Quadratic Programming (QP)** optimization scheme to act as a safety filter for a nominal controller[cite: 8, 9].

## 📌 Project Overview
[cite_start]The primary challenge addressed is the simultaneous management of conflicting objectives: preventing collisions (with static obstacles and other agents) while maintaining network connectivity among swarm members[cite: 7]. [cite_start]The method is validated using high-fidelity simulations of the **Crazyflie 2.1** nano-quadrotor dynamic model[cite: 10, 13].

### Key Features
* [cite_start]**Decentralized Navigation:** Agents navigate toward a target without a centralized computer[cite: 27].
* [cite_start]**Safety Guarantees:** Provides formal mathematical guarantees of forward invariance for the safe set[cite: 36].
* [cite_start]**Topological Elasticity:** Allows the swarm to deform its shape to pass through narrow gaps while maintaining minimal connectivity[cite: 248, 705].
* [cite_start]**Cascaded Control:** Separates high-level trajectory planning from low-level attitude stabilization[cite: 37, 77].

## ⚙️ System Architecture
[cite_start]The framework follows a three-layer cascaded structure[cite: 37]:

1.  [cite_start]**High-Level Guidance:** A nominal PD controller generates a reference acceleration ($u_{nom}$) towards the goal[cite: 38, 85, 86].
2.  [cite_start]**Safety Filter (QP-CBF):** A local Quadratic Program modifies the nominal input to satisfy HO-CBF constraints for safety and connectivity[cite: 39, 40].
3.  [cite_start]**Low-Level Control:** Safe accelerations are mapped to thrust and attitude commands (roll, pitch) tracked by a PID/PD controller[cite: 41, 120, 137].

### Mathematical Formulation
[cite_start]The safety filter solves the following optimization problem at each time step[cite: 104]:

[cite_start]$$u_{safe}^* = \min_{u \in \mathbb{R}^3} \frac{1}{2} ||u - u_{nom}||^2$$ [cite: 106]

**Subject to:**
* [cite_start]**Safety Barrier (Repulsive):** $\dot{h}_s + \gamma_1 \dot{h}_s + \gamma_2 h_s \ge 0$, where $h_s(x_i) = ||p_i - p_{obs}||^2 - R_{safe}^2 \ge 0$[cite: 90, 96].
* [cite_start]**Connectivity Barrier (Attractive):** $\dot{h}_c + \gamma_1 \dot{h}_c + \gamma_2 h_c \ge 0$, where $h_c(x_i) = R_c^2 - ||p_i - p_j||^2 \ge 0$[cite: 97, 102].
* **Actuation Limits:** $||u_i|| [cite_start]\le u_{max}$[cite: 65, 66].

## 🚀 Simulation Results
[cite_start]The system was validated in **MATLAB/Simulink R2025b** with a swarm of $N=4$ agents[cite: 165, 168].

* [cite_start]**Case Study A (Balanced):** The swarm successfully negotiated a cluster of 6 obstacles, splitting and regrouping while maintaining connectivity links below $R_c = 1.0$ m[cite: 170, 196, 202, 246].
* [cite_start]**Case Study B (Stress Test):** Even with under-damped tuning ($K_d=0.8$) causing oscillations, the CBF filter successfully clamped the states to prevent all collisions[cite: 409, 411, 465, 707].
* [cite_start]**Case Study C (Deadlock):** In a perfectly symmetric scenario where the obstacle size exceeds the communication range, the swarm prioritized safety and connectivity, "freezing" in a safe state rather than breaking formation[cite: 530, 531, 540, 708].

## 🛠️ Requirements
* [cite_start]**Software:** MATLAB / Simulink R2025b[cite: 165].
* **Toolboxes:** Control System Toolbox, Optimization Toolbox (for QP solver).
* [cite_start]**Hardware Model:** Bitcraze Crazyflie 2.1 parameters[cite: 172].

## 👥 Authors
* [cite_start]**Alessia Iacono** [cite: 3]
* [cite_start]**Pietro Gennarelli** [cite: 3]
* [cite_start]**Institution:** Università degli Studi di Napoli Federico II [cite: 15]
* [cite_start]**Course:** Field and Service Robotics [cite: 19]

## 📜 References
1.  A. D. Ames, et al. [cite_start]"Control Barrier Functions: Theory and Applications," *Proceedings of the IEEE*, 2019[cite: 721].
2.  G. Notomista and M. Egerstedt. [cite_start]"Constraint-Driven Coordinated Control of Multi-Robot Systems," *ACC*, 2018[cite: 723].
3.  L. Wang, et al. [cite_start]"Safety Barrier Certificates for Collisions-Free Multi-Robot Systems," *IEEE Transactions on Robotics*, 2017[cite: 724].
