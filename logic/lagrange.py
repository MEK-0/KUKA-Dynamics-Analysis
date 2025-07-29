import numpy as np
from sympy import symbols, Matrix, diff, simplify, sin, cos, Function

def calculate_lagrange():
    # Zaman ve değişkenler
    t = symbols('t')
    theta1, theta2 = symbols('theta1 theta2', cls=Function)
    theta1 = theta1(t)
    theta2 = theta2(t)

    # Kinematik parametreler
    l1, l2 = symbols('l1 l2')  # link uzunlukları
    m1, m2 = symbols('m1 m2')  # kütleler
    I1, I2 = symbols('I1 I2')  # atalet momentleri
    g = symbols('g')           # yerçekimi ivmesi

    # Pozisyonlar
    x1 = l1/2 * cos(theta1)
    y1 = l1/2 * sin(theta1)
    x2 = l1 * cos(theta1) + l2/2 * cos(theta1 + theta2)
    y2 = l1 * sin(theta1) + l2/2 * sin(theta1 + theta2)

    # Hızlar
    dx1 = diff(x1, t)
    dy1 = diff(y1, t)
    dx2 = diff(x2, t)
    dy2 = diff(y2, t)

    # Kinetik enerji
    T1 = (1/2) * m1 * (dx1**2 + dy1**2) + (1/2) * I1 * diff(theta1, t)**2
    T2 = (1/2) * m2 * (dx2**2 + dy2**2) + (1/2) * I2 * (diff(theta1, t) + diff(theta2, t))**2
    T = simplify(T1 + T2)

    # Potansiyel enerji
    V1 = m1 * g * y1
    V2 = m2 * g * y2
    V = simplify(V1 + V2)

    # Lagrangian
    L = T - V

    # Generalized coordinates
    q = Matrix([theta1, theta2])
    dq = Matrix([diff(theta1, t), diff(theta2, t)])
    ddq = Matrix([diff(theta1, (t, 2)), diff(theta2, (t, 2))])

    # Lagrange denklemleri
    tau = []
    for i in range(2):
        dL_dqi = diff(L, q[i])
        dL_ddqi = diff(L, dq[i])
        dt_dL_ddqi = diff(dL_ddqi, t)
        equation = simplify(dt_dL_ddqi - dL_dqi)
        tau.append(equation)

    return tau

def calculate_lagrange_numerical(m1=1.0, m2=1.0, l1=1.0, l2=1.0, g=9.81):
    """
    Calculate Lagrange equations with numerical values for plotting
    """
    import numpy as np
    from sympy import symbols, sin, cos, lambdify
    
    # Time variable
    t = symbols('t')
    
    # Create time array
    time = np.linspace(0, 10, 100)
    
    # Simple example: constant angular velocities
    omega1 = 1.0  # rad/s
    omega2 = 0.5  # rad/s
    
    # Angular positions
    theta1 = omega1 * time
    theta2 = omega2 * time
    
    # Calculate torques using simplified equations
    # τ1 = m1*g*l1/2*sin(θ1) + m2*g*l1*sin(θ1) + m2*g*l2/2*sin(θ1+θ2)
    # τ2 = m2*g*l2/2*sin(θ1+θ2)
    
    tau1 = m1 * g * l1/2 * np.sin(theta1) + m2 * g * l1 * np.sin(theta1) + m2 * g * l2/2 * np.sin(theta1 + theta2)
    tau2 = m2 * g * l2/2 * np.sin(theta1 + theta2)
    
    return time, tau1, tau2
