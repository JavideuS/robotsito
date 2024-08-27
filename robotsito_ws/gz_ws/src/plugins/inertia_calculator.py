import numpy as np

# Given values
mass = float(input("Mass (kg): "))
a = float(input("Width (x axis) (m): "))
b = float(input("Length (y axis) (m): "))
c = float(input("Depth (z axis) (m): "))

# Calculate principal moments of inertia
Ixx = (1/12) * mass * (b**2 + c**2)
Iyy = (1/12) * mass * (a**2 + c**2)
Izz = (1/12) * mass * (a**2 + b**2)

rotation = input("Does it have rotation (y/n): ")

if rotation == "y" or rotation == "Y" :

    # Initial inertia tensor (aligned with principal axes)
    I = np.array([
     [Ixx, 0, 0],
     [0, Iyy, 0],
     [0, 0, Izz]
    ])

    rotation_axis = input("Rotation axis (x/y/z): ")
    angle = float(input("Angle of rotation in degrees: "))

    # Rotation matrix about the x-axis by angle
    theta = np.deg2rad(angle)

    if rotation_axis == "x" or rotation_axis == "X":

        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])

        # Rotate the inertia tensor
        I_rotated = Rx @ I @ Rx.T

    elif rotation_axis == "y" or rotation_axis == "Y":
        # Rotation matrix about the y-axis by angle
        Ry = np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])
        # Rotate the inertia tensor
        I_rotated = Ry @ I @ Ry.T

    elif rotation_axis == "z" or rotation_axis == "Z":
        # Rotation matrix about the z-axis by angle
        Rz = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
        # Rotate the inertia tensor
        I_rotated = Rz @ I @ Rz.T

    else:
        print("Invalid rotation axis selected.")
        I_rotated = I  # No rotation applied

    # Extract the values
    Ixx_r, Ixy_r, Ixz_r = I_rotated[0]
    Iyx_r, Iyy_r, Iyz_r = I_rotated[1]
    Izx_r, Izy_r, Izz_r = I_rotated[2]

    # SDF code snippet
    sdf_snippet = f"""
    <inertia>
    <ixx>{Ixx_r}</ixx>
    <ixy>{Ixy_r}</ixy>
    <ixz>{Ixz_r}</ixz>
    <iyy>{Iyy_r}</iyy>
    <iyz>{Iyz_r}</iyz>
    <izz>{Izz_r}</izz>
    </inertia>
    """

else:
    # SDF code snippet
    sdf_snippet = f"""
    <inertia>
    <ixx>{Ixx}</ixx>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyy>{Iyy}</iyy>
    <iyz>0</iyz>
    <izz>{Izz}</izz>
    </inertia>
    """

print(sdf_snippet)