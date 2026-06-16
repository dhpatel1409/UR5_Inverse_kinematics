import numpy as np

# link_lenght = a_i-1
# link_twist = alpha_i-1
# link_offset= d_i
# joint_angle=theta_i

def tranformation_matrix(a , alpha , d , theta):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    T = np.array([[cos_theta,-sin_theta,0,alpha],
                  [sin_theta*cos_alpha,cos_theta*cos_alpha,-sin_alpha,-d*sin_alpha],
                  [sin_theta*sin_alpha,cos_theta*sin_alpha,cos_alpha,d*cos_alpha],
                  [0,0,0,1]])
    
    return T

def forward_kinematics(q):
    #returns transformation matrix of end effector
    # Typical UR5 values in meters:
    d1 = 0.089159
    a2 = -0.425
    a3 = -0.39225
    d4 = 0.10915
    d5 = 0.09465
    d6 = 0.0823
    
    # Individual transformation matrices for base to particular frames
    T0_1 = tranformation_matrix(0, 0, d1, q[0])
    T1_2 = tranformation_matrix(0 , np.pi/2 ,  0,  q[1])
    T0_2=T0_1 @ T1_2
    T2_3 = tranformation_matrix(a2,0, 0,  q[2])
    T0_3=T0_2@T2_3
    T3_4 = tranformation_matrix(a3 ,0, d4, q[3])
    T0_4=T0_3@T3_4
    T4_5 = tranformation_matrix(0 , -np.pi/2,  d5, q[4])
    T0_5=T0_4@T4_5
    T5_6 = tranformation_matrix(0, -np.pi/2 ,  d6 , q[5])
    # T0_0=T0_5@T5_6
    
    T0_6 = T0_1 @ T1_2 @ T2_3 @ T3_4 @ T4_5 @ T5_6
    # frames = [np.eye(4), T0_1, T0_2, T0_3, T0_4, T0_5, T0_6]
    frames = [ T0_1, T0_2, T0_3, T0_4, T0_5, T0_6]
    return T0_6 , frames

def jacobian(frames):
    
    J = np.zeros((6, 6))  #colums for joint and row for linear velocity & angular velocity
    P_e = frames[-1][:3, 3] # Global 3D position vector of End-Effector tip
    
    for i in range(6):
        # Finds direction of joint rotation axis
        z_direction = frames[i][:3, 2]
        # vector from origine(base) to frame origin
        P_i = frames[i][:3, 3]
        
        # Linear velocity row component: J_v = z_i-1 x (P_e - P_i-1) as v=w cross r(distance)

        J[:3, i] = np.cross(z_direction, (P_e - P_i))

        # Angular velocity row component: J_omega = z_i-1

        J[3:, i] = z_direction
        
    return J

def compute_pose_error(T_desired, T_current):
    
    #Calculates the 6x1 spatial correction vector (Position error + Rotation error).
    
    # Linear Cartesian delta error [dx, dy, dz]

    pos_err = T_desired[:3, 3] - T_current[:3, 3]
    
    # Angular tracking mapping error using skew-symmetric cross products of column axes
    R_d = T_desired[:3, :3]
    R_c = T_current[:3, :3]
    
    rot_err = 0.5 * (np.cross(R_c[:, 0], R_d[:, 0]) + 
                     np.cross(R_c[:, 1], R_d[:, 1]) + 
                     np.cross(R_c[:, 2], R_d[:, 2]))
    
    return np.concatenate((pos_err, rot_err))

def inverse_kinematics(T_desired, q_guess=None, max_iters=200, tol=1e-6):
    
    #Iteratively determines joint vectors for a target pose matrix T_desired using Newton-Raphson pseudo-inverse optimization updates.
    
    if q_guess is None:
        # Balanced non-zero array to avoid starting in home joint singularity traps
        q_guess = np.array([0.1, -1.0, 1.0, 0.1, 0.1, 0.1])
        
    q = np.array(q_guess)
    
    for _ in range(max_iters):
        T_current, frames = forward_kinematics(q)
        error = compute_pose_error(T_desired, T_current)
        
        # Stop condition when error vector falls inside target tolerance
        if np.linalg.norm(error) < tol:
            return q, True 
            
        J = jacobian(frames)
        
        # Moore-Penrose pseudo-inverse handles matrix columns near singularities smoothly
        delta_q = np.linalg.pinv(J) @ error
        q += delta_q
        
    return q, False
# --- Verification Test Case ---
# if __name__ == "__main__":
#     # Test with a specific joint vector (e.g., all zeros / home position)
#     # test_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
#     T_end_effector = forward_kinematics(test_q)
    
#     print("Overall Transformation Matrix (T0_6):\n", T_end_effector)
#     print("\nEnd-Effector Position:") 

#     #considering 6th frame origin as end effector position
#     print(f"X: {T_end_effector[0,3]:.4f} m")
#     print(f"Y: {T_end_effector[1,3]:.4f} m")
#     print(f"Z: {T_end_effector[2,3]:.4f} m")

def run_verification_pipeline(q_test_case):
    #Executes the comprehensive verification pipeline. Uses target joint vector -> generates T_desired -> computes IK -> verifies delta error.
   
    print(f"1. Input Test Joint Vector (q_true):\n   {q_test_case}\n")
    
    # Step A: Forward Kinematics to find target objective position from joint angles
    T_desired, _ = forward_kinematics(q_test_case)
    print(f"2. Desired End-Effector Matrix Target (T_desired):\n{T_desired}\n")
    
    # Step B: Run Inverse Kinematics solver engine
    q_calculated, success = inverse_kinematics(T_desired)
    
    if not success:
        print("[-] Verification Error: Inverse Kinematics failed to find valid angles.")
        return False
        
    print(f"3. Output Calculated Joint Vector (q_calculated):\n   {q_calculated}\n")
    
    # Step C: Re-verify calculated joint angles back through FK
    T_verified, _ = forward_kinematics(q_calculated)
    
    print(f"2. calculated End-Effector Matrix Target (T_calculated):\n{T_verified}\n")
    # Step D: Quantify absolute tracking error gap
    absolute_matrix_error = np.linalg.norm(T_desired - T_verified)
    
    
    print(f"Convergence Success Status:  {success}")
    print(f"Absolute Validation Error:   {absolute_matrix_error:.4e}")
    
    # Validation constraint logic criteria [cite: 42]
    if absolute_matrix_error < 1e-5:
        print("STATUS FLAG:                [VALID IK SOLUTION]") 
    else:
        print("STATUS FLAG:                [INVALID SOLUTION ERROR]")


if __name__ == "__main__":
    # Test case configuration: Arbitrary sample posture values in radians
    sample_test_joints = np.array([0.5, -1.2, 0.8, -0.5, 1.5, 0.2])
    run_verification_pipeline(sample_test_joints)