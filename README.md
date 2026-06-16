# UR5_Inverse_kinematics
# UR5 Industrial Robotic Arm: Inverse Kinematics Verification Tool

An end-to-end Python-based kinematics engine designed to model, compute, and cross-verify the forward and inverse kinematics of a 6-axis Universal Robots UR5 manipulator.

## 📌 Project Overview
This project builds a mathematically rigorous verification pipeline to control a 6-DoF serial robot arm using structural parameters. Instead of using black-box libraries, the entire coordinate frame transformation map, geometric Jacobian, and optimization loops are engineered from first principles.

##  Tasks
- **Task 1: DH Parameter Modeling:** Formulated structural parameters using Craig's Modified Denavit-Hartenberg convention to map individual transformation matrices between consecutive joint frames.
- **Task 2: Forward Kinematics (FK):** Implemented compounding matrix cascades to compute exact 3D tip poses from input joint angles.
- **Task 3: Inverse Kinematics (IK) :** Built a numerical optimization solver using the Newton-Raphson method. Includes structural robustness via the Moore-Penrose pseudo-inverse.
- **Task 4:Verification Pipeline:** Validates computed IK vectors back through the FK pipeline to verify numerical precision down to a tolerance threshold of $10^{-5}$.
