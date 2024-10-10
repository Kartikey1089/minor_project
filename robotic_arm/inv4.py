import numpy as np

# Link lengths for inverse kinematics
L1 = 120  # Length from shoulder to elbow
L2 = 120  # Length from elbow to wrist
L3 = 35   # Length for wrist to gripper (considered in calculations)

# Function to calculate inverse kinematics
def inverse_kinematics(x, y, z):
    # Calculate the base angle
    theta_base = np.degrees(np.arctan2(y, x))

    # Calculate the projection on the xy-plane and distance from shoulder to point
    r = np.sqrt(x**2 + y**2) - L3
    d = np.sqrt(r**2 + z**2)

    # Check if the point is reachable
    if d > (L1 + L2):
        print("Target is unreachable.")
        return None

    # Calculate shoulder and elbow angles using trigonometry
    A = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    if A < -1 or A > 1:
        print("Invalid configuration for shoulder angle.")
        return None
    theta_shoulder = np.degrees(np.arccos(A))

    B = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    if B < -1 or B > 1:
        print("Invalid configuration for elbow angle.")
        return None
    theta_elbow = np.degrees(np.arccos(B))

    # Calculate pitch angle based on the vertical position
    theta_pitch = np.degrees(np.arctan2(z, r))

    return theta_base, theta_shoulder, theta_elbow, theta_pitch

# Example usage
if __name__ == "__main__":
    x = float(input("Enter the X coordinate (mm): "))
    y = float(input("Enter the Y coordinate (mm): "))
    z = float(input("Enter the Z coordinate (mm): "))

    angles = inverse_kinematics(x, y, z)
    if angles is not None:
        print(f"Base angle: {angles[0]:.2f}°")
        print(f"Shoulder angle: {angles[1]:.2f}°")
        print(f"Elbow angle: {angles[2]:.2f}°")
        print(f"Pitch angle: {angles[3]:.2f}°")
