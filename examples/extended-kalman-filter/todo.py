def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F @ x + B @ u

    return x


def observation_model(x):
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = H @ x

    return z

def ekf_estimation(x_estimated, PEst, z, u):
    #  Predict
    xPred = motion_model(x_estimated, u)
    jF = jacob_f(x_estimated, u)
    PPred = jF @ PEst @ jF.T + Q

    #  Update
    jH = jacob_h()
    zPred = observation_model(xPred)
    y = z - zPred
    S = jH @ PPred @ jH.T + R
    K = PPred @ jH.T @ np.linalg.inv(S)
    x_estimated = xPred + K @ y
    PEst = (np.eye(len(x_estimated)) - K @ jH) @ PPred
    return x_estimated, PEst


def main():

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        x_real, z, xDR, ud = observation(x_real, xDR, u)

        x_estimated, PEst = ekf_estimation(x_estimated, PEst, z, ud)