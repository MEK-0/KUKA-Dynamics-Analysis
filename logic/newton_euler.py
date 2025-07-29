# logic/newton_euler.py

from logic.inertia import calculate_inertia_rod

def calculate_newton_euler_torque(mass: float, length: float, angular_acceleration: float) -> float:
    """
    Calculates torque using Newton-Euler formula: τ = I * α
    """
    inertia = calculate_inertia_rod(mass, length)
    torque = inertia * angular_acceleration
    return round(torque, 4)
