import numpy as np
from forward_kinematics import modDH


def localInWindow(ang, lo, hi):
    cands = np.array([ang, ang + 360, ang - 360])
    mask = (cands >= lo) & (cands <= hi)
    vals = cands[mask]
    return vals


def fitToWindow(ang, lo, hi):
    cands = np.array([ang, ang + 360, ang - 360])
    vals = cands[(cands >= lo) & (cands <= hi)]
    return vals


T_target = np.array(
    [
        [  -0.935752464576754,  -0.107277337898905,   0.335944783873564,  756.191893889629],
        [   0.113896830134094,  -0.993492582803418,    0,       0        ],
        [   0.333758651009883,   0.0382630459832821,  0.941881681628932, 1075.48001648],
        [   0,       0,       0,         1],

    ]
)

DHM = np.array(
    [[0, 0, 400], [-90, 25, 0], [0, 560, 0], [-90, 25, 515], [90, 0, 0], [-90, 0, 90]]
)

alpha = DHM[:, 0]
a = DHM[:, 1]
d = DHM[:, 2]

th_offset = np.array([0, 0, -90, 0, 0, 180])
th_inv = np.array([-1, 1, 1, -1, 1, -1])

lim = np.array(
    [[-170, 170], [-190, 45], [-120, 156], [-185, 185], [-120, 120], [-350, 350]]
)

G = np.eye(4)
H = modDH(np.array([0]), np.array([0]), np.array([d[5]]), np.array([0]))

T_w = T_target @ np.linalg.inv(H)
x_w = T_w[0, 3]
y_w = T_w[1, 3]
z_w = T_w[2, 3]

d1 = d[0]
a2 = a[1]
a3 = a[2]
a4 = a[3]
d4 = d[3]

L1 = a3
L2 = np.hypot(a4, d4)

delta = np.degrees(np.arctan2(d4, a4))

th1_raw = np.degrees(np.arctan2(y_w, x_w))
th1_used_sols = np.array([th1_raw, th1_raw + 180])

sols123 = []

for th1_used in th1_used_sols:
    x1 = np.cos(np.radians(th1_used)) * x_w + np.sin(np.radians(th1_used)) * y_w
    z1 = z_w

    X = x1 - a2
    Z = z1 - d1

    R_abs = np.hypot(X, Z)
    if R_abs < 1e-9:
        continue

    R_th = np.degrees(np.arctan2(-Z, X))

    c_alpha = (L1**2 + R_abs**2 - L2**2) / (2 * L1 * R_abs)
    c_alpha = np.clip(c_alpha, -1, 1)
    alpha_tri = np.degrees(np.arccos(c_alpha))

    th2_used_list = np.array([R_th + alpha_tri, R_th - alpha_tri])

    c_gamma = (L1**2 + L2**2 - R_abs**2) / (2 * L1 * L2)
    c_gamma = np.clip(c_gamma, -1, 1)
    gamma = np.degrees(np.arccos(c_gamma))

    th3_down = (180 - gamma) - delta
    th3_up = -(180 - gamma) - delta
    th3_used_list = np.array([th3_up, th3_down])

    sets = np.array(
        [
            [th1_used, th2_used_list[0], th3_used_list[0]],
            [th1_used, th2_used_list[1], th3_used_list[1]],
        ]
    )

    for k in range(2):
        th1u_k = sets[k, 0]
        th2u_k = sets[k, 1]
        th3u_k = sets[k, 2]

        q1_raw = np.round((th1u_k - th_offset[0]) / th_inv[0], 3)
        q2_raw = np.round((th2u_k - th_offset[1]) / th_inv[1], 3)
        q3_raw = np.round((th3u_k - th_offset[2]) / th_inv[2], 3)

        q1_list = localInWindow(q1_raw, lim[0, 0], lim[0, 1])
        q2_list = localInWindow(q2_raw, lim[1, 0], lim[1, 1])
        q3_list = localInWindow(q3_raw, lim[2, 0], lim[2, 1])

        if len(q1_list) == 0 or len(q2_list) == 0 or len(q3_list) == 0:
            continue

        q1 = q1_list[0]
        q2 = q2_list[0]
        q3 = q3_list[0]

        sols123.append([q1, q2, q3])

sols123 = np.array(sols123)

Thfinal = []

for i in range(len(sols123)):
    q1 = sols123[i, 0]
    q2 = sols123[i, 1]
    q3 = sols123[i, 2]

    th1 = th_inv[0] * q1 + th_offset[0]
    th2 = th_inv[1] * q2 + th_offset[1]
    th3 = th_inv[2] * q3 + th_offset[2]

    T03 = modDH(a[0:3], alpha[0:3], d[0:3], np.array([th1, th2, th3]))
    T03 = T03 @ modDH(
        np.array([a[3]]), np.array([alpha[3]]), np.array([d[3]]), np.array([0])
    )
    T_orient = np.linalg.inv(T03) @ np.linalg.inv(G) @ T_target @ np.linalg.inv(H)

    c5 = T_orient[2, 2]
    th5_p = np.degrees(np.arccos(c5))
    th5_n = -np.degrees(np.arccos(c5))

    th5u_list = np.array([th5_p, th5_n])

    for th5u in th5u_list:
        s5 = np.sin(np.radians(th5u))

        if abs(s5) < 1e-7:
            th4u = 0
            th6u = np.degrees(np.arctan2(T_orient[0, 0], T_orient[0, 1]))
        else:
            th4u = np.degrees(np.arctan2(-T_orient[1, 2] / s5, (-T_orient[0, 2]) / s5))
            th6u = np.degrees(np.arctan2((-T_orient[2, 1]) / s5, (T_orient[2, 0]) / s5))

        q4_raw = (th4u - th_offset[3]) / th_inv[3]
        q5_raw = (th5u - th_offset[4]) / th_inv[4]
        q6_raw = (th6u - th_offset[5]) / th_inv[5]

        q5_list = fitToWindow(q5_raw, lim[4, 0], lim[4, 1])
        if len(q5_list) == 0:
            continue
        q5 = q5_list[0]

        q4_cands = np.array([q4_raw, q4_raw + 360, q4_raw - 360])
        q4_list = np.unique(q4_cands[(q4_cands >= lim[3, 0]) & (q4_cands <= lim[3, 1])])
        if len(q4_list) == 0:
            continue

        q6_cands = np.array([q6_raw, q6_raw + 360, q6_raw - 360])
        q6_list = np.unique(q6_cands[(q6_cands >= lim[5, 0]) & (q6_cands <= lim[5, 1])])
        if len(q6_list) == 0:
            continue

        for q4 in q4_list:
            for q6 in q6_list:
                Thfinal.append([q1, q2, q3, q4, q5, q6])

Thfinal = np.array(Thfinal)

np.set_printoptions(precision=7, suppress=True, linewidth=200)

Thfinal_unique = np.unique(np.round(Thfinal, 3), axis=0)

print("\n" + "=" * 80)
print("UNIQUE VALID SOLUTIONS (8)")
print("=" * 80)
print("All IK solutions [th1 th2 th3 th4 th5 th6] (RoboDK deg):")
print(Thfinal_unique)

print("\n" + "=" * 80)
print("ALL VALID SOLUTIONS WITH ANGLE VARIANTS (16)")
print("=" * 80)
print("All IK solutions [th1 th2 th3 th4 th5 th6] (RoboDK deg):")
print(Thfinal)

print("\n" + "=" * 80)
print("VERIFICATION - FK CHECK")
print("=" * 80)

inv_joint = np.array([-1, 1, 1, -1, 1, -1])

failed_count = 0
for i in range(len(Thfinal)):
    ths = Thfinal[i, :]

    DHM_check = np.array(
        [
            [0, 0, 400, inv_joint[0] * ths[0]],
            [-90, 25, 0, inv_joint[1] * ths[1]],
            [0, 560, 0, inv_joint[2] * ths[2] - 90],
            [-90, 25, 515, inv_joint[3] * ths[3]],
            [90, 0, 0, inv_joint[4] * ths[4]],
            [-90, 0, 90, inv_joint[5] * ths[5] + 180],
        ]
    )

    alpha_i = DHM_check[:, 0]
    a_i = DHM_check[:, 1]
    d_i = DHM_check[:, 2]
    theta_i = DHM_check[:, 3]

    T_attempt = modDH(a_i, alpha_i, d_i, theta_i)

    T_attempt_rounded = np.round(T_attempt, 1)
    T_target_rounded = np.round(T_target, 1)

    if np.allclose(T_attempt_rounded, T_target_rounded, atol=1e-10):
        print(f"Solution {i + 1}: PASS ✓")
    else:
        failed_count += 1
        print(f"Solution {i + 1}: FAIL ✗")
        print(f"  Thetas: {ths}")
        print(f"  T_attempt:\n{T_attempt}")
        print(f"  T_target:\n{T_target}")

print("\n" + "=" * 80)
print(f"SUMMARY: {len(Thfinal) - failed_count}/{len(Thfinal)} solutions verified")
print("=" * 80)
