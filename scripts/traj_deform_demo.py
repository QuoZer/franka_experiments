import numpy as np
import matplotlib.pyplot as plt


def traj_gen(dt=0.01, tr_type='cubic', frequency = 1):
    # Initial and final conditions
    theta_i = 0  # Initial position
    theta_f = 1  # Final position
    theta_dot_i = 0  # Initial velocity
    theta_dot_f = 0  # Final velocity
    t_f = 3/2*np.pi  # Final time

    # Generate a vector of time points
    t = np.arange(0, t_f, dt)
    if tr_type=='cubic':
        # Compute the coefficients of the cubic polynomial
        a = 2 * theta_i - 2 * theta_f + theta_dot_f * t_f + theta_dot_i * t_f
        b = -3 * theta_i + 3 * theta_f - 2 * theta_dot_i * t_f - theta_dot_f * t_f
        c = theta_dot_i * t_f
        d = theta_i

        # Compute the trajectory
        theta = a * (t / t_f) ** 3 + b * (t / t_f) ** 2 + c * (t / t_f) + d
    elif tr_type=='sine':
        theta = 0.75 * np.sin(t)  # Position

    return t, theta

def human_force(t, amplitude, start_time, time_length):
    end_time = start_time + time_length
    # Generate saw signal
    stepSignal = np.zeros_like(t)
    stepSignal[t < start_time] = 0 
    stepSignal[t >= start_time] = amplitude 
    stepSignal[t > end_time] = 0 

    return stepSignal

def get_deform_vector(N, method):
    if method == 0:
        A = np.zeros((N+2, N))
        # Set the diagonal elements
        np.fill_diagonal(A[1:], 1)
        np.fill_diagonal(A[:, 1:], -2)
        if N > 1:
            np.fill_diagonal(A[:, 2:], 1)
        # Compute R and H
        R = np.dot(A.transpose(), A)
        H = np.linalg.inv(R)
    elif method == 1:
        # Initialize a matrix filled with zeros
        A = np.zeros((N+3, N))

        # Fill diagonals (min jerk diff matrix)
        np.fill_diagonal(A, 1)
        np.fill_diagonal(A[1:], -3)
        np.fill_diagonal(A[2:], 3)
        np.fill_diagonal(A[3:], -1)

        # Compute R
        R = np.dot(A.transpose(), A)

        # Initialize matrix B
        B = np.zeros((4, N))
        B[0, 0] = 1
        B[1, 1] = 1
        B[2, N - 2] = 1
        B[3, N - 1] = 1

        # Compute G
        I = np.eye(N)  # Identity matrix
        unit = np.ones(N)  # Unit vector 
        R_inv = np.linalg.inv(R)
        BR_inv = np.dot(B, R_inv)
        G = np.dot(I - np.dot(R_inv, np.dot(B.transpose(), np.dot(np.linalg.inv(np.dot(BR_inv, B.transpose())), B))), np.dot(R_inv, unit))

        # Compute H
        H = np.sqrt(N) * G / np.linalg.norm(G)

    return H

def deform_trajectory(t, theta, step_signal, def_length, admittance, method=0):
    N = def_length
    H = get_deform_vector(N, method)
    
    deformed_theta = theta.copy()
    deformation_segment = np.zeros(N)
    tracked_theta = theta.copy()    

    plt.plot(H, )
    plt.title('Deformation shape')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.grid(True)

    for i in range(len(t)-1):
        sample_time = t[i+1] - t[i]
        Uh = np.zeros(N)
        Uh[0] = step_signal[i]
        if method == 0:
            deformation_segment = admittance * np.dot(H, Uh)
        elif method == 1:
            deformation_segment = admittance * sample_time * H * step_signal[i]
        # perform the addition
        nN = min(N, len(t) - i)
        deformed_theta[i:i + nN] += deformation_segment[:nN]

    return deformed_theta

if __name__ == '__main__':
    admittance = 2
    deform_len = 50
    t, theta = traj_gen(0.01, 'sine', 1)
    step_signal = human_force(t, -0.5, 1, 0.6)
    deformed_theta = deform_trajectory(t, theta, step_signal, deform_len, admittance, method=1)
    
    # Plot the trajectory
    plt.figure()
    plt.plot(t, theta)
    plt.title('Robot Joint Trajectory')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.grid(True)
    # plt.show()
    # Plot
    #plt.figure()
    plt.plot(t, step_signal)
    plt.title('Step Signal')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.grid(True)
    # plt.show()
    # Plot the deformed trajectory
    # plt.figure()
    plt.plot(t, deformed_theta)
    plt.title('Deformed Robot Joint Trajectory')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.grid(True)
    plt.show()
