# robots/kuka_dynamics.py

import numpy as np
from typing import List, Tuple, Dict
from .kuka_robots import KukaRobot, get_robot_by_name

def calculate_kuka_newton_euler(robot_name: str, joint_angles: List[float], 
                               joint_velocities: List[float], joint_accelerations: List[float]) -> List[float]:
    """
    Calculate joint torques for KUKA robot using Newton-Euler method
    
    Args:
        robot_name: Name of the KUKA robot model
        joint_angles: List of joint angles in radians
        joint_velocities: List of joint velocities in rad/s
        joint_accelerations: List of joint accelerations in rad/s²
    
    Returns:
        List of joint torques in Nm
    """
    robot = get_robot_by_name(robot_name)
    if robot is None:
        raise ValueError(f"Robot {robot_name} not found")
    
    # Simplified Newton-Euler calculation for KUKA robots
    torques = []
    
    for i, (angle, velocity, acceleration) in enumerate(zip(joint_angles, joint_velocities, joint_accelerations)):
        link = robot.links[i]
        
        # Inertia torque: τ = I * α
        inertia_torque = link.inertia * acceleration
        
        # Gravitational torque (simplified)
        # τ_g = m * g * l * sin(θ)
        gravitational_torque = link.mass * 9.81 * link.center_of_mass * np.sin(angle)
        
        # Centrifugal and Coriolis effects (simplified)
        centrifugal_torque = 0.5 * link.mass * link.length**2 * velocity**2
        
        # Total torque for this joint
        total_torque = inertia_torque + gravitational_torque + centrifugal_torque
        torques.append(total_torque)
    
    return torques

def calculate_kuka_lagrange(robot_name: str, joint_angles: List[float], 
                           joint_velocities: List[float], joint_accelerations: List[float]) -> List[float]:
    """
    Calculate joint torques for KUKA robot using Lagrange method
    
    Args:
        robot_name: Name of the KUKA robot model
        joint_angles: List of joint angles in radians
        joint_velocities: List of joint velocities in rad/s
        joint_accelerations: List of joint accelerations in rad/s²
    
    Returns:
        List of joint torques in Nm
    """
    robot = get_robot_by_name(robot_name)
    if robot is None:
        raise ValueError(f"Robot {robot_name} not found")
    
    # Simplified Lagrange calculation for KUKA robots
    torques = []
    
    for i, (angle, velocity, acceleration) in enumerate(zip(joint_angles, joint_velocities, joint_accelerations)):
        link = robot.links[i]
        
        # Mass matrix element (simplified)
        M = link.inertia
        
        # Coriolis matrix element (simplified)
        C = 0.1 * link.mass * link.length**2 * velocity
        
        # Gravity vector element
        G = link.mass * 9.81 * link.center_of_mass * np.cos(angle)
        
        # Lagrange equation: M*q_ddot + C*q_dot + G = τ
        torque = M * acceleration + C * velocity + G
        torques.append(torque)
    
    return torques

def calculate_kuka_kinetic_energy(robot_name: str, joint_velocities: List[float]) -> float:
    """
    Calculate total kinetic energy of KUKA robot
    
    Args:
        robot_name: Name of the KUKA robot model
        joint_velocities: List of joint velocities in rad/s
    
    Returns:
        Total kinetic energy in Joules
    """
    robot = get_robot_by_name(robot_name)
    if robot is None:
        raise ValueError(f"Robot {robot_name} not found")
    
    kinetic_energy = 0.0
    
    for i, velocity in enumerate(joint_velocities):
        link = robot.links[i]
        # Kinetic energy: K = 0.5 * I * ω²
        kinetic_energy += 0.5 * link.inertia * velocity**2
    
    return kinetic_energy

def calculate_kuka_potential_energy(robot_name: str, joint_angles: List[float]) -> float:
    """
    Calculate total potential energy of KUKA robot
    
    Args:
        robot_name: Name of the KUKA robot model
        joint_angles: List of joint angles in radians
    
    Returns:
        Total potential energy in Joules
    """
    robot = get_robot_by_name(robot_name)
    if robot is None:
        raise ValueError(f"Robot {robot_name} not found")
    
    potential_energy = 0.0
    height = 0.0
    
    for i, angle in enumerate(joint_angles):
        link = robot.links[i]
        # Accumulate height for each link
        height += link.length * np.cos(angle)
        # Potential energy: U = m * g * h
        potential_energy += link.mass * 9.81 * height
    
    return potential_energy

def calculate_kuka_jacobian(robot_name: str, joint_angles: List[float]) -> np.ndarray:
    """
    Calculate Jacobian matrix for KUKA robot (simplified)
    
    Args:
        robot_name: Name of the KUKA robot model
        joint_angles: List of joint angles in radians
    
    Returns:
        Jacobian matrix (6x6 for 6-DOF robot)
    """
    robot = get_robot_by_name(robot_name)
    if robot is None:
        raise ValueError(f"Robot {robot_name} not found")
    
    # Simplified Jacobian calculation
    # In a real implementation, this would use DH parameters
    J = np.zeros((6, 6))
    
    for i in range(6):
        for j in range(6):
            if i == j:
                # Diagonal elements (simplified)
                J[i, j] = robot.links[i].length * np.cos(joint_angles[i])
            else:
                # Off-diagonal elements (simplified)
                J[i, j] = 0.1 * robot.links[i].length * np.sin(joint_angles[i])
    
    return J

def calculate_kuka_workspace_torques(robot_name: str, time_points: int = 100) -> Tuple[np.ndarray, List[np.ndarray]]:
    """
    Calculate torques over time for KUKA robot workspace analysis
    
    Args:
        robot_name: Name of the KUKA robot model
        time_points: Number of time points for analysis
    
    Returns:
        Tuple of (time_array, list_of_torque_arrays)
    """
    robot = get_robot_by_name(robot_name)
    if robot is None:
        raise ValueError(f"Robot {robot_name} not found")
    
    time = np.linspace(0, 10, time_points)  # 10 seconds simulation
    
    # Generate sample joint trajectories
    joint_angles = []
    joint_velocities = []
    joint_accelerations = []
    
    for i in range(robot.dof):
        # Simple sinusoidal motion for each joint
        freq = 0.5 + i * 0.1  # Different frequency for each joint
        angle = 0.5 * np.sin(freq * time)
        velocity = 0.5 * freq * np.cos(freq * time)
        acceleration = -0.5 * freq**2 * np.sin(freq * time)
        
        joint_angles.append(angle)
        joint_velocities.append(velocity)
        joint_accelerations.append(acceleration)
    
    # Calculate torques for each time point
    newton_euler_torques = []
    lagrange_torques = []
    
    for t in range(time_points):
        angles = [joint_angles[i][t] for i in range(robot.dof)]
        velocities = [joint_velocities[i][t] for i in range(robot.dof)]
        accelerations = [joint_accelerations[i][t] for i in range(robot.dof)]
        
        ne_torque = calculate_kuka_newton_euler(robot_name, angles, velocities, accelerations)
        lag_torque = calculate_kuka_lagrange(robot_name, angles, velocities, accelerations)
        
        newton_euler_torques.append(ne_torque)
        lagrange_torques.append(lag_torque)
    
    return time, [np.array(newton_euler_torques), np.array(lagrange_torques)]

def get_kuka_robot_info(robot_name: str) -> Dict:
    """
    Get comprehensive information about a KUKA robot
    
    Args:
        robot_name: Name of the KUKA robot model
    
    Returns:
        Dictionary containing robot information
    """
    robot = get_robot_by_name(robot_name)
    if robot is None:
        raise ValueError(f"Robot {robot_name} not found")
    
    # Calculate some basic dynamics
    zero_angles = [0.0] * robot.dof
    zero_velocities = [0.0] * robot.dof
    zero_accelerations = [0.0] * robot.dof
    
    static_torques = calculate_kuka_newton_euler(robot_name, zero_angles, zero_velocities, zero_accelerations)
    
    info = {
        'name': robot.name,
        'model': robot.model,
        'dof': robot.dof,
        'max_payload': robot.max_payload,
        'reach': robot.reach,
        'repeatability': robot.repeatability,
        'max_speed': robot.max_speed,
        'total_mass': sum(link.mass for link in robot.links),
        'total_inertia': sum(link.inertia for link in robot.links),
        'static_torques': static_torques,
        'links': [
            {
                'mass': link.mass,
                'length': link.length,
                'inertia': link.inertia,
                'center_of_mass': link.center_of_mass,
                'joint_type': link.joint_type
            }
            for link in robot.links
        ]
    }
    
    return info 