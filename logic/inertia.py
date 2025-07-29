# logic/inertia.py

def calculate_inertia_rod(mass: float, length: float) -> float:
    """
    Moment of inertia for a rod rotating around one end.
    I = (1/3) * m * L^2
    """
    return (1 / 3) * mass * length ** 2
