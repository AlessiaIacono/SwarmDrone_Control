# Distributed Swarm Control with Connectivity Maintenance and Obstacle Avoidance

This project implements a decentralized control framework for the safe navigation of a micro-UAV swarm in cluttered 3D environments.The architecture utilizes **High-Order Control Barrier Functions (HO-CBFs)** embedded within a **Quadratic Programming (QP)** optimization scheme to act as a safety filter for a nominal controller.

## 📌 Project Overview
The primary challenge addressed is the simultaneous management of conflicting objectives: preventing collisions (with static obstacles and other agents) while maintaining network connectivity among swarm members. The method is validated using high-fidelity simulations of the **Crazyflie 2.1** nano-quadrotor dynamic model.

### Key Features
**Decentralized Navigation:** Agents navigate toward a target without a centralized computer.
**Safety Guarantees:** Provides formal mathematical guarantees of forward invariance for the safe set.
**Topological Elasticity:** Allows the swarm to deform its shape to pass through narrow gaps while maintaining minimal connectivity.
**Cascaded Control:** Separates high-level trajectory planning from low-level attitude stabilization.

## ⚙️ System Architecture
The framework follows a three-layer cascaded structure:

1.  **High-Level Guidance:** A nominal PD controller generates a reference acceleration ($u_{nom}$) towards the goal.
2.  **Safety Filter (QP-CBF):** A local Quadratic Program modifies the nominal input to satisfy HO-CBF constraints for safety and connectivity.
3.  **Low-Level Control:** Safe accelerations are mapped to thrust and attitude commands (roll, pitch) tracked by a PID/PD controller.

### Mathematical Formulation
The safety filter solves the following optimization problem at each time step:

$$u_{safe}^* = \min_{u \in \mathbb{R}^3} \frac{1}{2} ||u - u_{nom}||^2$$ 

**Subject to:**
**Safety Barrier (Repulsive):** $\dot{h}_s + \gamma_1 \dot{h}_s + \gamma_2 h_s \ge 0$, where $h_s(x_i) = ||p_i - p_{obs}||^2 - R_{safe}^2 \ge 0$.
* **Connectivity Barrier (Attractive):** $\dot{h}_c + \gamma_1 \dot{h}_c + \gamma_2 h_c \ge 0$, where $h_c(x_i) = R_c^2 - ||p_i - p_j||^2 \ge 0$[cite: 97, 102].
* **Actuation Limits:** $||u_i|| \le u_{max}$.

## 🚀 Simulation Results
The system was validated in **MATLAB/Simulink R2025b** with a swarm of $N=4$ agents.

**Case Study A (Balanced):** The swarm successfully negotiated a cluster of 6 obstacles, splitting and regrouping while maintaining connectivity links below $R_c = 1.0$ m.
**Case Study B (Stress Test):** Even with under-damped tuning ($K_d=0.8$) causing oscillations, the CBF filter successfully clamped the states to prevent all collisions.
**Case Study C (Deadlock):** In a perfectly symmetric scenario where the obstacle size exceeds the communication range, the swarm prioritized safety and connectivity, "freezing" in a safe state rather than breaking formation.

## 🛠️ Requirements
* **Software:** MATLAB / Simulink R2025b.
* **Toolboxes:** Control System Toolbox, Optimization Toolbox (for QP solver).
* **Hardware Model:** Bitcraze Crazyflie 2.1 parameters.

## 👥 Authors
**Alessia Iacono** 
**Pietro Gennarelli** 
**Institution:** Università degli Studi di Napoli Federico II
**Course:** Field and Service Robotics 

## 📜 References
1.  A. D. Ames, et al. "Control Barrier Functions: Theory and Applications," *Proceedings of the IEEE*, 2019.
2.  G. Notomista and M. Egerstedt. "Constraint-Driven Coordinated Control of Multi-Robot Systems," *ACC*, 2018.
3.  L. Wang, et al. "Safety Barrier Certificates for Collisions-Free Multi-Robot Systems," *IEEE Transactions on Robotics*, 2017.
