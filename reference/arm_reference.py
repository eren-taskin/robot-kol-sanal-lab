"""
Reference model of the OpenMANIPULATOR-X arm (KON 445E project).

Independent Python implementation of SPEC.md, used ONLY to produce
reference numbers (test_vectors.json) that the C# tests compare against.
It is not part of the application.

Two independent routes to the inertia matrix are computed and compared:
  (a) the closed forms of the literature review, eqs. (14)-(17)
  (b) the general sum over bodies, eq. (12), with Jacobians by finite differences
and two routes to gravity:
  (a) eq. (18)
  (b) the gradient of the potential energy V, by finite differences.

Run:  python3 -I arm_reference.py  ->  writes test_vectors.json and prints checks
"""
import json, math
import numpy as np

# ---------------------------------------------------------------- parameters
D1 = 0.0595
L2 = math.hypot(0.024, 0.128)
BETA = math.atan2(0.128, 0.024)
L3 = 0.124
L4 = 0.126

# body i = 1..4 : mass (kg), ell (m), eps (rad), Ixx, Iyy, Izz, Ixz (kg m^2)
BODIES = [
    dict(m=0.0984, ell=0.0474, eps=math.radians(-90.4), Ixx=34.5e-6, Iyy=32.7e-6, Izz=18.9e-6, Ixz=-0.4e-6),
    dict(m=0.1385, ell=0.1022, eps=math.radians(-84.2), Ixx=330.6e-6, Iyy=342.9e-6, Izz=60.3e-6, Ixz=-38.5e-6),
    dict(m=0.1327, ell=0.0909, eps=math.radians(-0.1), Ixx=30.7e-6, Iyy=242.3e-6, Izz=251.6e-6, Ixz=-0.3e-6),
    dict(m=0.1433, ell=0.0451, eps=math.radians(-11.4), Ixx=80.9e-6, Iyy=76.0e-6, Izz=93.1e-6, Ixz=-1.0e-6),
]

PLANETS = {"Earth": 9.81, "Moon": 1.62, "Mars": 3.71, "HighG": 24.79, "Orbit": 0.0}


def with_payload(bodies, mp):
    """Rigidly attach a point mass mp (kg) at the tool point to body 4 (SPEC 6.2)."""
    if mp == 0.0:
        return [dict(b) for b in bodies]
    b = dict(bodies[3])
    c4 = np.array([b["ell"] * math.cos(b["eps"]), 0.0, -b["ell"] * math.sin(b["eps"])])
    tp = np.array([L4, 0.0, 0.0])
    m = b["m"] + mp
    c = (b["m"] * c4 + mp * tp) / m
    I = np.array([[b["Ixx"], 0, b["Ixz"]], [0, b["Iyy"], 0], [b["Ixz"], 0, b["Izz"]]])

    def shift(mass, d):
        return mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))

    I = I + shift(b["m"], c4 - c) + shift(mp, tp - c)
    nb = dict(m=m, ell=math.hypot(c[0], c[2]), eps=math.atan2(-c[2], c[0]),
              Ixx=I[0, 0], Iyy=I[1, 1], Izz=I[2, 2], Ixz=I[0, 2])
    out = [dict(x) for x in bodies]
    out[3] = nb
    return out


def thetas(q):
    return [0.0, q[1], q[1] + q[2], q[1] + q[2] + q[3]]


# ---------------------------------------------------------------- kinematics
def fk(q):
    t = thetas(q)
    r = L2 * math.cos(t[1] - BETA) + L3 * math.cos(t[2]) + L4 * math.cos(t[3])
    z = D1 - L2 * math.sin(t[1] - BETA) - L3 * math.sin(t[2]) - L4 * math.sin(t[3])
    return np.array([r * math.cos(q[0]), r * math.sin(q[0]), z]), t[3]


def ik(p, psi, elbow_up=True):
    x, y, z = p
    q1 = math.atan2(y, x)
    u = math.hypot(x, y) - L4 * math.cos(psi)
    v = z - D1 + L4 * math.sin(psi)
    c = (u * u + v * v - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if abs(c) > 1:
        return None
    phi = -math.acos(c) if elbow_up else math.acos(c)
    e = math.atan2(v, u) - math.atan2(L3 * math.sin(phi), L2 + L3 * math.cos(phi))
    q2 = BETA - e
    q3 = -BETA - phi
    q4 = psi - q2 - q3
    return np.array([q1, q2, q3, q4])


def jacobian_planar(q):
    t = thetas(q)
    s2, c2 = L2 * math.sin(t[1] - BETA), L2 * math.cos(t[1] - BETA)
    s3, c3 = L3 * math.sin(t[2]), L3 * math.cos(t[2])
    s4, c4 = L4 * math.sin(t[3]), L4 * math.cos(t[3])
    return np.array([[-(s2 + s3 + s4), -(s3 + s4), -s4],
                     [-(c2 + c3 + c4), -(c3 + c4), -c4],
                     [1.0, 1.0, 1.0]])


def planar_rz(q):
    t = thetas(q)
    r = L2 * math.cos(t[1] - BETA) + L3 * math.cos(t[2]) + L4 * math.cos(t[3])
    z = D1 - L2 * math.sin(t[1] - BETA) - L3 * math.sin(t[2]) - L4 * math.sin(t[3])
    return np.array([r, z, t[3]])


# ---------------------------------------------------------------- body geometry
def com_rho_z(q, bodies):
    """horizontal distance rho_i and height z_i of each centre of mass"""
    t = thetas(q)
    out = []
    j_r = [0.0, 0.0, L2 * math.cos(t[1] - BETA), L2 * math.cos(t[1] - BETA) + L3 * math.cos(t[2])]
    j_z = [0.0, D1, D1 - L2 * math.sin(t[1] - BETA), D1 - L2 * math.sin(t[1] - BETA) - L3 * math.sin(t[2])]
    for i, b in enumerate(bodies):
        a = t[i] + b["eps"]
        out.append((j_r[i] + b["ell"] * math.cos(a), j_z[i] - b["ell"] * math.sin(a)))
    return out


def com_pos(q, bodies):
    return [np.array([rho * math.cos(q[0]), rho * math.sin(q[0]), z]) for rho, z in com_rho_z(q, bodies)]


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# ---------------------------------------------------------------- dynamics
def M_general(q, bodies, eps=1e-6):
    """eq. (12): sum of m Jv^T Jv + Jw^T R I R^T Jw, Jv by central differences"""
    n = 4
    M = np.zeros((n, n))
    t = thetas(q)
    Jv = [np.zeros((3, n)) for _ in range(4)]
    for k in range(n):
        dq = np.zeros(n); dq[k] = eps
        pp, pm = com_pos(q + dq, bodies), com_pos(q - dq, bodies)
        for i in range(4):
            Jv[i][:, k] = (pp[i] - pm[i]) / (2 * eps)
    ey = Rz(q[0]) @ np.array([0, 1, 0])
    for i, b in enumerate(bodies):
        Jw = np.zeros((3, n))
        Jw[:, 0] = [0, 0, 1]
        for j in range(1, i + 1):
            Jw[:, j] = ey
        R = Rz(q[0]) @ Ry(t[i])
        Ib = np.array([[b["Ixx"], 0, b["Ixz"]], [0, b["Iyy"], 0], [b["Ixz"], 0, b["Izz"]]])
        M += b["m"] * Jv[i].T @ Jv[i] + Jw.T @ R @ Ib @ R.T @ Jw
    return M


def M_closed(q, bodies):
    """eqs. (14)-(17)"""
    t = thetas(q)
    m = [b["m"] for b in bodies]; l = [b["ell"] for b in bodies]; e = [b["eps"] for b in bodies]
    rho = [r for r, _ in com_rho_z(q, bodies)]
    m11 = sum(bodies[i]["m"] * rho[i] ** 2 + bodies[i]["Ixx"] * math.sin(t[i]) ** 2
              + bodies[i]["Izz"] * math.cos(t[i]) ** 2
              - 2 * bodies[i]["Ixz"] * math.sin(t[i]) * math.cos(t[i]) for i in range(4))
    q3, q4 = q[2], q[3]
    J2, J3, J4 = bodies[1]["Iyy"], bodies[2]["Iyy"], bodies[3]["Iyy"]
    m2, m3, m4 = m[1], m[2], m[3]
    l2, l3, l4 = l[1], l[2], l[3]
    h = L2 * l3 * math.cos(q3 + BETA + e[2])
    k23 = L2 * L3 * math.cos(q3 + BETA)
    k24 = L2 * l4 * math.cos(q3 + q4 + BETA + e[3])
    k34 = L3 * l4 * math.cos(q4 + e[3])
    M44 = J4 + m4 * l4 ** 2
    M34 = J4 + m4 * (l4 ** 2 + k34)
    M33 = J3 + J4 + m3 * l3 ** 2 + m4 * (L3 ** 2 + l4 ** 2 + 2 * k34)
    M24 = J4 + m4 * (l4 ** 2 + k34 + k24)
    M23 = J3 + J4 + m3 * (l3 ** 2 + h) + m4 * (L3 ** 2 + l4 ** 2 + 2 * k34 + k23 + k24)
    M22 = (J2 + J3 + J4 + m2 * l2 ** 2 + m3 * (L2 ** 2 + l3 ** 2 + 2 * h)
           + m4 * (L2 ** 2 + L3 ** 2 + l4 ** 2 + 2 * k23 + 2 * k24 + 2 * k34))
    M = np.zeros((4, 4))
    M[0, 0] = m11
    M[1, 1], M[2, 2], M[3, 3] = M22, M33, M44
    M[1, 2] = M[2, 1] = M23
    M[1, 3] = M[3, 1] = M24
    M[2, 3] = M[3, 2] = M34
    return M


def potential(q, bodies, g):
    return g * sum(b["m"] * z for b, (_, z) in zip(bodies, com_rho_z(q, bodies)))


def gravity_closed(q, bodies, g):
    """eq. (18)"""
    t = thetas(q)
    m = [b["m"] for b in bodies]; l = [b["ell"] for b in bodies]; e = [b["eps"] for b in bodies]
    g4 = -g * m[3] * l[3] * math.cos(t[3] + e[3])
    g3 = g4 - g * (m[2] * l[2] * math.cos(t[2] + e[2]) + m[3] * L3 * math.cos(t[2]))
    g2 = g3 - g * (m[1] * l[1] * math.cos(t[1] + e[1]) + (m[2] + m[3]) * L2 * math.cos(t[1] - BETA))
    return np.array([0.0, g2, g3, g4])


def gravity_fd(q, bodies, g, eps=1e-6):
    out = np.zeros(4)
    for k in range(4):
        dq = np.zeros(4); dq[k] = eps
        out[k] = (potential(q + dq, bodies, g) - potential(q - dq, bodies, g)) / (2 * eps)
    return out


def coriolis_vector(q, qd, bodies, eps=1e-6):
    """C(q,qd) qd from Christoffel symbols, dM/dq by central differences"""
    dM = []
    for k in range(4):
        dq = np.zeros(4); dq[k] = eps
        dM.append((M_closed(q + dq, bodies) - M_closed(q - dq, bodies)) / (2 * eps))
    cv = np.zeros(4)
    for k in range(4):
        s = 0.0
        for i in range(4):
            for j in range(4):
                cijk = 0.5 * (dM[i][k, j] + dM[j][k, i] - dM[k][i, j])
                s += cijk * qd[i] * qd[j]
        cv[k] = s
    return cv


def f(x, tau, bodies, g):
    q, qd = x[:4], x[4:]
    M = M_closed(q, bodies)
    b = tau - coriolis_vector(q, qd, bodies) - gravity_closed(q, bodies, g)
    qdd = np.linalg.solve(M, b)
    return np.concatenate([qd, qdd])


def rk4(x, tau, h, bodies, g):
    k1 = f(x, tau, bodies, g)
    k2 = f(x + 0.5 * h * k1, tau, bodies, g)
    k3 = f(x + 0.5 * h * k2, tau, bodies, g)
    k4 = f(x + h * k3, tau, bodies, g)
    return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def energy(x, bodies, g):
    q, qd = x[:4], x[4:]
    return 0.5 * qd @ M_closed(q, bodies) @ qd + potential(q, bodies, g)


def simulate(x0, T, h, bodies, g):
    x = np.array(x0, float)
    n = int(round(T / h))
    for _ in range(n):
        x = rk4(x, np.zeros(4), h, bodies, g)
    return x


# ---------------------------------------------------------------- run checks
if __name__ == "__main__":
    rng = np.random.default_rng(445)
    poses = [np.zeros(4), np.array([0.3, 0.30, 0.25, 0.45]), np.array([-1.2, -0.5, 0.9, -0.3])]
    poses += [rng.uniform([-math.pi, -1.5, -1.5, -1.7], [math.pi, 1.5, 1.4, 2.0]) for _ in range(5)]

    worst_M = worst_g = 0.0
    pose_vectors = []
    for payload in (0.0, 0.5):
        bodies = with_payload(BODIES, payload)
        for q in poses:
            Mc, Mg = M_closed(q, bodies), M_general(q, bodies)
            # closed form neglects nothing here because the data has no out-of-plane terms
            worst_M = max(worst_M, np.max(np.abs(Mc - Mg)) / np.max(np.abs(Mc)))
            gc, gf = gravity_closed(q, bodies, 9.81), gravity_fd(q, bodies, 9.81)
            worst_g = max(worst_g, np.max(np.abs(gc - gf)))
            assert np.all(np.linalg.eigvalsh(Mc) > 0)
            p, psi = fk(q)
            pose_vectors.append(dict(payload_kg=payload, q=q.tolist(), tool_position=p.tolist(), tool_pitch=psi,
                                     M=Mc.tolist(), gravity_earth=gc.tolist()))
    print(f"M closed vs general: worst relative diff {worst_M:.2e}")
    print(f"g closed vs dV/dq  : worst abs diff {worst_g:.2e} N m")

    # inverse kinematics round trip
    ik_vectors, worst_ik = [], 0.0
    for q in [np.array([0.3, 0.30, 0.25, 0.45]), np.array([1.0, -0.2, 0.6, 0.4]), np.array([-0.7, 0.5, -0.2, 0.9])]:
        p, psi = fk(q)
        qi = ik(p, psi, elbow_up=True)
        qo = ik(p, psi, elbow_up=False)
        p2, _ = fk(qi)
        worst_ik = max(worst_ik, np.max(np.abs(p2 - p)))
        ik_vectors.append(dict(target_position=p.tolist(), target_pitch=psi, elbow_up=qi.tolist(),
                               elbow_down=qo.tolist()))
    print(f"IK -> FK position error: {worst_ik:.2e} m")

    # planar Jacobian vs finite difference
    q = np.array([0.3, 0.30, 0.25, 0.45]); Jfd = np.zeros((3, 3))
    for k in range(3):
        dq = np.zeros(4); dq[k + 1] = 1e-6
        Jfd[:, k] = (planar_rz(q + dq) - planar_rz(q - dq)) / 2e-6
    print(f"Jacobian vs FD: {np.max(np.abs(Jfd - jacobian_planar(q))):.2e}")
    detJ = np.linalg.det(jacobian_planar(q)); detF = -L2 * L3 * math.sin(q[2] + BETA)
    print(f"det J = {detJ:.6e}, formula (9) = {detF:.6e}")

    # free motion: energy and vertical angular momentum
    x0 = [0.3, 0.30, 0.25, 0.45, 1.5, 0.5, -0.8, 1.0]
    bodies = BODIES
    E0 = energy(np.array(x0), bodies, 9.81)
    x = np.array(x0, float); h = 2e-3; Emax = 0.0; p1 = []
    for _ in range(200):
        x = rk4(x, np.zeros(4), h, bodies, 9.81)
        Emax = max(Emax, abs(energy(x, bodies, 9.81) - E0))
        p1.append((M_closed(x[:4], bodies) @ x[4:])[0])
    print(f"free motion 0.4 s, h=2 ms: max |E-E0| = {Emax:.2e} J, p1 drift = {max(p1) - min(p1):.2e}")

    # convergence: error at T against a fine reference
    T = 0.4
    ref = simulate(x0, T, 1e-4, bodies, 9.81)
    e1 = np.max(np.abs(simulate(x0, T, 4e-3, bodies, 9.81) - ref))
    e2 = np.max(np.abs(simulate(x0, T, 2e-3, bodies, 9.81) - ref))
    e3 = np.max(np.abs(simulate(x0, T, 1e-3, bodies, 9.81) - ref))
    print(f"RK4 error ratios: {e1 / e2:.1f}, {e2 / e3:.1f}  (expect ~16)")
    print(f"error at h=1 ms vs reference: {e3:.2e}")

    # statics and torque budget per planet
    # arm reaching horizontally at the height of joint 2, tool level, 0.38 m from the base axis
    horiz = ik(np.array([0.38, 0.0, D1]), 0.0)
    torque_budget = {}
    for name, g in PLANETS.items():
        for mp in (0.0, 0.5):
            torque_budget[f"{name}_payload{mp}"] = gravity_closed(horiz, with_payload(BODIES, mp), g).tolist() if horiz is not None else None
    print("horizontal pose", None if horiz is None else np.round(horiz, 4))
    for k, v in torque_budget.items():
        if v: print(f"  {k:22s} max|g| = {max(abs(t) for t in v):.3f} N m")

    traj = {T_: simulate(x0, T_, 1e-4, bodies, 9.81).tolist() for T_ in (0.1, 0.2, 0.4)}

    out = dict(
        description="Reference values for KON 445E arm model. SI units, angles in rad. "
                    "Generated by reference/arm_reference.py from SPEC.md equations.",
        geometry=dict(d1=D1, L2=L2, beta=BETA, L3=L3, L4=L4),
        bodies=BODIES,
        planets=PLANETS,
        poses=pose_vectors,
        inverse_kinematics=ik_vectors,
        free_motion=dict(planet="Earth", g=9.81, x0=x0, energy0=E0,
                         reference_states={str(k): v for k, v in traj.items()},
                         note="reference integrated with RK4 h=1e-4 s; C# with h=1e-3 s should match within 5e-5 (measured difference at h=1 ms: see error_h1ms_vs_reference)",
                         error_h1ms_vs_reference=e3),
        horizontal_reach_pose=None if horiz is None else horiz.tolist(),
        gravity_torque_horizontal_reach=torque_budget,
        stall_torque=4.1,
    )
    with open("test_vectors.json", "w") as fh:
        json.dump(out, fh, indent=1)
    print("wrote test_vectors.json")
