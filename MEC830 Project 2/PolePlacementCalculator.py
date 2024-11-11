import numpy as np

# System parameters
g = 9.81                # Gravity (m/s^2)
m = 0.05                # Mass of pendulum (kg)
M = 0.250                 # Mass of cart (kg)
l = 0.43                 # Length of pendulum (m)
b = 0.001                # Pivot damping coefficient

# Linearized state-space matrix A
A = np.array([
    [0, 1, 0, 0],
    [(M + m) * g / (M * l), -(M + m) * b / (M * m * l**2), 0, 0],
    [0, 0, 0, 1],
    [m * l**2 / (M + m), 0, 0, 0]
])

# Calculate and print the eigenvalues of A
eigenvalues = np.linalg.eigvals(A)
print("Eigenvalues of A:", eigenvalues)
