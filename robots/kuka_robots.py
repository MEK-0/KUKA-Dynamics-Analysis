# robots/kuka_robots.py

import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Tuple

@dataclass
class RobotLink:
    """Robot link parameters"""
    mass: float  # kg
    length: float  # m
    inertia: float  # kg*m²
    center_of_mass: float  # m (distance from joint)
    joint_type: str  # 'revolute' or 'prismatic'

@dataclass
class KukaRobot:
    """KUKA robot configuration"""
    name: str
    model: str
    dof: int  # degrees of freedom
    links: List[RobotLink]
    max_payload: float  # kg
    reach: float  # m
    repeatability: float  # mm
    max_speed: float  # rad/s
    
    def get_dh_parameters(self) -> List[Dict]:
        """Get Denavit-Hartenberg parameters for the robot"""
        # This would contain the actual DH parameters for each KUKA model
        # For now, returning a placeholder structure
        dh_params = []
        for i, link in enumerate(self.links):
            dh_params.append({
                'a': link.length,  # link length
                'alpha': 0,  # link twist
                'd': 0,  # link offset
                'theta': 0,  # joint angle
                'joint_type': link.joint_type
            })
        return dh_params
    
    def get_mass_matrix(self) -> np.ndarray:
        """Get mass matrix for the robot"""
        masses = [link.mass for link in self.links]
        return np.diag(masses)
    
    def get_inertia_matrix(self) -> np.ndarray:
        """Get inertia matrix for the robot"""
        inertias = [link.inertia for link in self.links]
        return np.diag(inertias)

# KUKA Robot Definitions
KUKA_KR3_R540 = KukaRobot(
    name="KR3 R540",
    model="KR3 R540",
    dof=6,
    links=[
        RobotLink(mass=3.5, length=0.25, inertia=0.1, center_of_mass=0.125, joint_type='revolute'),
        RobotLink(mass=8.2, length=0.56, inertia=0.25, center_of_mass=0.28, joint_type='revolute'),
        RobotLink(mass=2.4, length=0.035, inertia=0.05, center_of_mass=0.0175, joint_type='revolute'),
        RobotLink(mass=1.9, length=0.0, inertia=0.03, center_of_mass=0.0, joint_type='revolute'),
        RobotLink(mass=0.5, length=0.0, inertia=0.01, center_of_mass=0.0, joint_type='revolute'),
        RobotLink(mass=0.3, length=0.0, inertia=0.005, center_of_mass=0.0, joint_type='revolute')
    ],
    max_payload=3.0,
    reach=0.541,
    repeatability=0.02,
    max_speed=3.14
)

KUKA_KR6_R900 = KukaRobot(
    name="KR6 R900",
    model="KR6 R900",
    dof=6,
    links=[
        RobotLink(mass=4.8, length=0.25, inertia=0.15, center_of_mass=0.125, joint_type='revolute'),
        RobotLink(mass=11.2, length=0.56, inertia=0.35, center_of_mass=0.28, joint_type='revolute'),
        RobotLink(mass=3.8, length=0.035, inertia=0.08, center_of_mass=0.0175, joint_type='revolute'),
        RobotLink(mass=2.5, length=0.0, inertia=0.04, center_of_mass=0.0, joint_type='revolute'),
        RobotLink(mass=0.8, length=0.0, inertia=0.015, center_of_mass=0.0, joint_type='revolute'),
        RobotLink(mass=0.4, length=0.0, inertia=0.008, center_of_mass=0.0, joint_type='revolute')
    ],
    max_payload=6.0,
    reach=0.901,
    repeatability=0.02,
    max_speed=2.62
)

KUKA_KR10_R1100 = KukaRobot(
    name="KR10 R1100",
    model="KR10 R1100",
    dof=6,
    links=[
        RobotLink(mass=6.2, length=0.25, inertia=0.2, center_of_mass=0.125, joint_type='revolute'),
        RobotLink(mass=15.8, length=0.56, inertia=0.45, center_of_mass=0.28, joint_type='revolute'),
        RobotLink(mass=5.2, length=0.035, inertia=0.12, center_of_mass=0.0175, joint_type='revolute'),
        RobotLink(mass=3.8, length=0.0, inertia=0.06, center_of_mass=0.0, joint_type='revolute'),
        RobotLink(mass=1.2, length=0.0, inertia=0.02, center_of_mass=0.0, joint_type='revolute'),
        RobotLink(mass=0.6, length=0.0, inertia=0.012, center_of_mass=0.0, joint_type='revolute')
    ],
    max_payload=10.0,
    reach=1.101,
    repeatability=0.02,
    max_speed=2.44
)

KUKA_KR16_R1610 = KukaRobot(
    name="KR16 R1610",
    model="KR16 R1610",
    dof=6,
    links=[
        RobotLink(mass=8.5, length=0.25, inertia=0.25, center_of_mass=0.125, joint_type='revolute'),
        RobotLink(mass=22.4, length=0.56, inertia=0.6, center_of_mass=0.28, joint_type='revolute'),
        RobotLink(mass=7.8, length=0.035, inertia=0.18, center_of_mass=0.0175, joint_type='revolute'),
        RobotLink(mass=5.2, length=0.0, inertia=0.08, center_of_mass=0.0, joint_type='revolute'),
        RobotLink(mass=1.8, length=0.0, inertia=0.025, center_of_mass=0.0, joint_type='revolute'),
        RobotLink(mass=0.9, length=0.0, inertia=0.015, center_of_mass=0.0, joint_type='revolute')
    ],
    max_payload=16.0,
    reach=1.61,
    repeatability=0.02,
    max_speed=2.18
)

# Available KUKA robots dictionary
KUKA_ROBOTS = {
    'KR3 R540': KUKA_KR3_R540,
    'KR6 R900': KUKA_KR6_R900,
    'KR10 R1100': KUKA_KR10_R1100,
    'KR16 R1610': KUKA_KR16_R1610
}

def get_available_robots() -> List[str]:
    """Get list of available KUKA robot models"""
    return list(KUKA_ROBOTS.keys())

def get_robot_by_name(name: str) -> KukaRobot:
    """Get KUKA robot configuration by name"""
    return KUKA_ROBOTS.get(name)

def calculate_robot_inertia(robot: KukaRobot) -> float:
    """Calculate total inertia of the robot"""
    total_inertia = sum(link.inertia for link in robot.links)
    return total_inertia

def calculate_robot_mass(robot: KukaRobot) -> float:
    """Calculate total mass of the robot"""
    total_mass = sum(link.mass for link in robot.links)
    return total_mass

def get_robot_specifications(robot: KukaRobot) -> Dict:
    """Get comprehensive robot specifications"""
    return {
        'name': robot.name,
        'model': robot.model,
        'dof': robot.dof,
        'max_payload': robot.max_payload,
        'reach': robot.reach,
        'repeatability': robot.repeatability,
        'max_speed': robot.max_speed,
        'total_mass': calculate_robot_mass(robot),
        'total_inertia': calculate_robot_inertia(robot),
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