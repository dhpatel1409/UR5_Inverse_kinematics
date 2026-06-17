# UR5_Inverse_kinematics
# UR5 Industrial Robotic Arm: Inverse Kinematics Verification Tool

An end-to-end Python-based kinematics engine designed to model, compute, and cross-verify the forward and inverse kinematics of a 6-axis Universal Robots UR5 manipulator.

## 📌 Project Overview
This project builds a mathematically rigorous verification pipeline to control a 6-DoF serial robot arm using structural parameters of UR5 Robot
<p align="center">
  <img src="UR5_Robot.png" width="500"></br>
  <em>Figure 1: UR5 Robot</em>
</p>
Instead of using black-box libraries, the entire coordinate frame transformation map, geometric Jacobian, and optimization loops are engineered from first principles.

##  Tasks
- **Task 1: DH Parameter Modeling:** Formulated structural parameters using
  <p align="center">
    <img src="D_H_parameter_model.png" width="500"></br>
    <em>D_H parameter of UR5</em>
  </p>
  to map individual transformation matrices between consecutive joint frames.
- **Task 2: Forward Kinematics (FK):** Implemented compounding matrix cascades to compute exact 3D tip poses from input joint angles.
- **Task 3: Inverse Kinematics (IK) :** Built a numerical optimization solver using the Newton-Raphson method. Includes structural robustness via the Moore-Penrose pseudo-inverse. Includes [jacobian matrix formulation](calculating_jacobian.png) used with Newton Raphson to find new joint parameters.
- **Task 4:Verification Pipeline:** Validates computed IK vectors back through the FK pipeline to verify numerical precision down to a tolerance threshold of $10^{-5}$.
  1.  Ground Truth Generation: Define a known arbitrary target joint vector configuration. 
  2.  Target Pose Generation: Pass ground truth through the verified Forward Kinematics engine to compute an exact target homogeneous matrix $T_{desired}$.
  3.  IK Execution Test: Feed only $T_{desired}$ into the Inverse Kinematics solver iterative loop, and calculate the resulting joint configuration solution $q_{calculated}$.
  4. Error Convergence Evaluation: Run $q_{calculated}$ back through Forward Kinematics to secure a verified pose matrix $T_{verified}$, and evaluate the absolute discrepancy matrix gap relative to $T_{desired}$. 
Absolute Evaluation Error = $\|T_{desired} - T_{verified}\|$

