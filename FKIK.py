import numpy as np

def rotation_z(theta):
    rad = np.radians(theta)
    return np.array([
        [np.cos(rad), -np.sin(rad), 0, 0],
        [np.sin(rad), np.cos(rad), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def transformation_x(a):
    return np.array([
        [1, 0, 0, a],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def transformation_z(b):
    return np.array([
        [1, 0, 0, b],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def forward_kinematics(length, theta):
    T = np.eye(4)
    for L, theta in zip(lengths, thetas):
        T = T @ rotation_z(theta) @ transformation_x(L)
    pos = T @ np.array([0, 0, 0, 1])
    return T, pos[:3] 

def inverse_kinematics_2d(x, y, L1, L2):
    r = np.sqrt(x**2 + y**2)
    if r > (L1 + L2):
        raise ValueError("Posisi di luar jangkauan lengan!")

    cos_theta2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = np.degrees(np.arccos(cos_theta2))

    k1 = L1 + L2 * np.cos(np.radians(theta2))
    k2 = L2 * np.sin(np.radians(theta2))
    theta1 = np.degrees(np.arctan2(y, x) - np.arctan2(k2, k1))
    return theta1, theta2

print("=== Robot Arm Kinematics 2-3 DoF ===")
mode = input("Pilih mode [1] Forward Kinematics  [2] Inverse Kinematics: ")

if mode == "1":
    n = int(input("Masukkan jumlah Degree of Freedom (2 atau 3): "))
    lengths = []
    thetas = []
    for i in range(n):
        L = float(input(f"Masukkan panjang lengan {i+1}: "))
        theta = float(input(f"Masukkan sudut servo {i+1} (derajat): "))
        lengths.append(L)
        thetas.append(theta)

    T, pos = forward_kinematics(lengths, thetas)
    print("\n=== Hasil Forward Kinematics ===")
    print("Matriks Homogen Total (4x4):")
    print(np.round(T, 4))
    print(f"\nKoordinat Ujung Efektor: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")

elif mode == "2":
    print("\n=== Inverse Kinematics (2 DoF planar) ===")
    L1 = float(input("Masukkan panjang lengan 1: "))
    L2 = float(input("Masukkan panjang lengan 2: "))
    x = float(input("Masukkan posisi x tujuan: "))
    y = float(input("Masukkan posisi y tujuan: "))

    try:
        t1, t2 = inverse_kinematics_2d(x, y, L1, L2)
        print(f"\nSudut servo yang diperlukan:")
        print(f"θ1 = {t1:.3f}°")
        print(f"θ2 = {t2:.3f}°")
    except ValueError as e:
        print("Error:", e)

else:
    print("Pilihan tidak valid.")