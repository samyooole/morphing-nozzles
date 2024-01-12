from mpopt import mp
import numpy as np
import casadi as ca


def run_ocp():

    ocp = mp.OCP(n_states=7, n_controls=4)

    # Initialize parameters
    Re = 6356000  # m
    omegaE = 7.29211585e-5
    rho0 = 1.225
    rhoH = 7200.0
    Sa = 4 * np.pi
    Cd = 0.5
    muE = 3.986012e14
    g0 = 9.80665
    Ve = 2300

    def spherical_to_cartesian(latitude, longitude, radius=6356000):
        # Convert latitude and longitude from degrees to radians
        lat_rad = np.radians(latitude)
        lon_rad = np.radians(longitude)
        
        # Calculate Cartesian coordinates
        x = radius * np.sin(lat_rad) * np.cos(lon_rad)
        y = radius * np.sin(lat_rad) * np.sin(lon_rad)
        z = radius * np.cos(lat_rad)
        
        return x, y, z

    #r0 = spherical_to_cartesian(45,81)

    # Variable initialization
    lat0 = 28.5 * np.pi / 180.0
    r0 = np.array([Re * np.cos(lat0), 0.0, Re * np.sin(lat0)]) # not very robust to starting position! maybe has got to do with final orbital parameters
    v0 = omegaE * np.array([-r0[1], r0[0], 0.0])
    m0 = 301454.0
    mf = 4164.0
    mdrySrb = 19290.0 - 17010.0
    mdryFirst = 104380.0 - 95550.0
    mdrySecond = 19300.0 - 16820.0
    x0 = np.array([r0[0], r0[1], r0[2], v0[0], v0[1], v0[2], m0])



    def dynamics(x,u,t, param=0):
        r = x[:3]
        v = x[3:6]
        m = x[6]

        massflow = u[0]
        unit_vector = u[1:4]
        

        r_mag =  ca.sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2])
        v_rel = ca.vertcat(v[0] + r[1] * omegaE, v[1] - r[0] * omegaE, v[2])
        v_rel_mag = ca.sqrt(v_rel[0] * v_rel[0] + v_rel[1] * v_rel[1] + v_rel[2] * v_rel[2]               + 0.001) #HWUC)

        h = r_mag - Re

        gravity_acceleration = - muE / (r_mag * r_mag * r_mag) * r


        F = massflow * Ve
        thrust_acceleration = F / m * unit_vector


        rho = rho0 * ca.exp(-h / rhoH)
        drag_acceleration = -rho / (2 * m) * Sa * Cd * v_rel_mag * v_rel

        xdot = [
            x[3],
            x[4],
            x[5],
            F / m * unit_vector[0] + drag_acceleration[0] * param + gravity_acceleration[0],
            F / m * unit_vector[1] + drag_acceleration[1] * param + gravity_acceleration[1],
            F / m * unit_vector[2] + drag_acceleration[2] * param + gravity_acceleration[2],
            -massflow,
        ]

        return xdot

    def get_dynamics(param):
        dynamics0 = lambda x, u, t: dynamics(x, u, t, param=param)
        return [dynamics0]


    ocp.dynamics = get_dynamics(0)



    # Step-2: Add path constraints
    def path_constraints(x, u, t):
        return [
            u[1] * u[1] + u[2] * u[2] + u[3] * u[3] - 1,
            -u[1] * u[1] - u[2] * u[2] - u[3] * u[3] + 1,
            -ca.sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]) / Re + 1,
        ]


    ocp.path_constraints = [path_constraints]

    # Step-3: Add terminal cost and constraints
    def terminal_cost3(xf, tf, x0, t0):
        return -xf[-1] / m0


    ocp.terminal_costs[0] = terminal_cost3


    def terminal_constraints3(x, t, x0, t0):
        # https://space.stackexchange.com/questions/1904/how-to-programmatically-calculate-orbital-elements-using-position-velocity-vecto
        # http://control.asu.edu/Classes/MAE462/462Lecture06.pdf
        h = ca.vertcat(
            x[1] * x[5] - x[4] * x[2], x[3] * x[2] - x[0] * x[5], x[0] * x[4] - x[1] * x[3]
        )

        n = ca.vertcat(-h[1], h[0], 0)
        r = ca.sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2])

        e = ca.vertcat(
            1 / muE * (x[4] * h[2] - x[5] * h[1]) - x[0] / r,
            1 / muE * (x[5] * h[0] - x[3] * h[2]) - x[1] / r,
            1 / muE * (x[3] * h[1] - x[4] * h[0]) - x[2] / r,
        )

        e_mag = ca.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2])
        h_sq = h[0] * h[0] + h[1] * h[1] + h[2] * h[2]
        v_mag = ca.sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5])

        a = -muE / (v_mag * v_mag - 2.0 * muE / r)
        i = ca.acos(h[2] / ca.sqrt(h_sq))
        n_mag = ca.sqrt(n[0] * n[0] + n[1] * n[1])

        node_asc = ca.acos(n[0] / n_mag)
        # if n[1] < -1e-12:
        node_asc = 2 * np.pi - node_asc

        argP = ca.acos((n[0] * e[0] + n[1] * e[1]) / (n_mag * e_mag))
        # if e[2] < 0:
        #    argP = 2*np.pi - argP

        a_req = 24361140.0
        e_req = 0.7308
        i_req = 28.5 * np.pi / 180.0
        node_asc_req = 269.8 * np.pi / 180.0
        argP_req = 130.5 * np.pi / 180.0

        return [
            (a - a_req) / (Re),
            e_mag - e_req,
            i - i_req,
            node_asc - node_asc_req,
            argP - argP_req,
        ]


    ocp.terminal_constraints[0] = terminal_constraints3

    ocp.scale_x = [
        1 / Re,
        1 / Re,
        1 / Re,
        1 / np.sqrt(muE / Re),
        1 / np.sqrt(muE / Re),
        1 / np.sqrt(muE / Re),
        1 / m0,
    ]
    ocp.scale_t = np.sqrt(muE / Re) / Re

    # Intial guess
    # Initial guess estimation
    def ae_to_rv(a, e, i, node, argP, th):
        p = a * (1.0 - e * e)
        r = p / (1.0 + e * np.cos(th))

        r_vec = np.array([r * np.cos(th), r * np.sin(th), 0.0])
        v_vec = np.sqrt(muE / p) * np.array([-np.sin(th), e + np.cos(th), 0.0])

        cn, sn = np.cos(node), np.sin(node)
        cp, sp = np.cos(argP), np.sin(argP)
        ci, si = np.cos(i), np.sin(i)

        R = np.array(
            [
                [cn * cp - sn * sp * ci, -cn * sp - sn * cp * ci, sn * si],
                [sn * cp + cn * sp * ci, -sn * sp + cn * cp * ci, -cn * si],
                [sp * si, cp * si, ci],
            ]
        )

        r_i = np.dot(R, r_vec)
        v_i = np.dot(R, v_vec)

        return r_i, v_i


    # Target conditions
    a_req = 24361140.0
    e_req = 0.7308
    i_req = 28.5 * np.pi / 180.0
    node_asc_req = 269.8 * np.pi / 180.0
    argP_req = 130.5 * np.pi / 180.0
    th = 0.0
    rf, vf = ae_to_rv(a_req, e_req, i_req, node_asc_req, argP_req, th)


    xf = np.array([rf[0], rf[1], rf[2], vf[0], vf[1], vf[2], mf])

    ocp.x00[0] = x0
    ocp.xf0[0] = xf
    ocp.u00[0] = [1,1,0,0]
    ocp.uf0[0] = [1,1,0,0]

    rmin, rmax = -2 * Re, 2 * Re
    vmin, vmax = -10000.0, 10000.0
    rvmin = [rmin, rmin, rmin, vmin, vmin, vmin]
    rvmax = [rmax, rmax, rmax, vmax, vmax, vmax]
    lbx = rvmin + [xf[-1]]
    ubx = rvmax + [x0[-1]]
    ocp.lbx = np.array([lbx])
    ocp.ubx = np.array([ubx])

    # Bounds for control inputs
    umin = [0, -1, -1, -1]
    umax = [m0, 1, 1, 1]
    ocp.lbu = np.array([umin])
    ocp.ubu = np.array([umax])


    ocp.validate()

    ocp.dynamics = get_dynamics(0)
    mpo = mp.mpopt(ocp, 5, 6)
    sol = mpo.solve()

    ocp.dynamics = get_dynamics(1)
    ocp.validate()

    mpo._ocp = ocp
    sol = mpo.solve(
        sol, reinitialize_nlp=True, nlp_solver_options={"ipopt.acceptable_tol": 1e-6}, plot=True
    )


    post = mpo.process_results(sol, plot=False, scaling=False)
    mp.post_process._INTERPOLATION_NODES_PER_SEG = 200
    x, u, t, _ = post.get_data(interpolate=True)

    return x,u,t
