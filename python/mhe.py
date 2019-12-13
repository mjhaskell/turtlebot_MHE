import numpy as np
from copy import deepcopy
from scipy.optimize import minimize
from extractdata import landmarks
from copy import deepcopy

def unwrap(phi):
    phi -= 2 * np.pi * np.floor((phi + np.pi) * 0.5/np.pi)
    return phi

class MHE:
    def __init__(self):
        self.pose_hist = []    
        self.z_hist = []       
        self.z_ind_hist = []

        self.mu = np.array([2.5,-1.7, 0.175])

        self.pose_hist.append(deepcopy(self.mu)) #change this so that the position is inisialized by the constructor
        self.Omega = np.diag([1e3, 1e3, 0.5e3])
        self.Omega2 = np.diag([1e3, 1e3, 0.5e3])
        self.R_inv = np.diag([1/(.35**2), 1/(.07**2)])
        self.N = 5  #Size of the window to optimize over

    def setParams(self, omega, sig_r, sig_phi):
        self.Omega = omega
        self.R_inv = np.diag([1/(sig_r**2), 1/(sig_phi**2)])

    def propagateState(self, state, v, w, dt):
        theta = state[2]
        st = np.sin(theta)
        ct = np.cos(theta)

        A = np.array([v * ct,
                    v * st,
                    w])
        temp = state + A * dt
        temp[2] = unwrap(temp[2])
        return temp

    def update(self, mu, z, z_ind, v, w, dt):
        mu_bar = self.propagateState(self.mu, v, w, dt)

        self.pose_hist.append(mu_bar)
        self.z_hist.append(z)
        self.z_ind_hist.append(z_ind)

        if len(self.pose_hist) >= self.N + 1: 
            mu_temp = self.optimize(self.pose_hist[-self.N:], self.z_hist[-self.N:], self.z_ind_hist[-self.N:])
            mu_temp = mu_temp.reshape((3, int(mu_temp.size/3)), order='F')
            mu_temp[2] = unwrap(mu_temp[2])
            self.mu = mu_temp[:,-1]
            for i in range(self.N):
                self.pose_hist[-(self.N - i)] = mu_temp[:,i]
        else:
            mu_temp = self.optimize(self.pose_hist, self.z_hist, self.z_ind_hist) # z_hist doesn't have the same length as pose_hist here b/c original position doesn't have
            mu_temp = mu_temp.reshape((3, int(mu_temp.size/3)), order='F')
            mu_temp[2] = unwrap(mu_temp[2])
            self.mu = mu_temp[:,-1]
            for i in range(int(mu_temp.size/3)):
                self.pose_hist[i] = mu_temp[:,i]

    def optimize(self, mu, z, z_ind):
        mu = np.array(mu).T.flatten(order='F')
        x0 = deepcopy(mu)
        sigma = np.array(sigma)

        x_hat_opt = minimize(self.objective_fun, mu, method='SLSQP', jac=False, args=(x0, z, z_ind, landmarks), options={'ftol':1e-5, 'disp':False})

        return x_hat_opt.x

    def objective_fun(self, mu, x0, z, z_ind, lms):
        e_x = 0
        e_z = 0

        dx = (x0 - mu).reshape((-1, 3, 1), order='F')
        dx[:,2] = unwrap(dx[:,2])
        e_x = np.sum(dx.transpose(0,2,1) @ self.Omega @ dx) #Hand tuned values

        # temp = mu.reshape((-1,3,1), order='F')
        # dx2 = np.diff(temp, axis=0)
        # temp2 = x0.reshape((-1,3,1),order='F')
        # dx0 = np.diff(temp2, axis=0)
        # diff = dx0 = dx2
        # diff[:,2] = unwrap(diff[:,2])
        # e_x2 = np.sum((dx0 - dx2).transpose(0,2,1) @ self.Omega2 @ (dx0 - dx2)) #Error between successive poses

        for i in range(len(z)):
            if z_ind[i].size > 0:
                z_hat = self.h(mu[3*i:3*i+3], lms, z_ind[i])
                dz = z[i] - z_hat
                dz[1] = unwrap(dz[1])
                e_z += np.sum(np.diagonal(dz.T @ self.R_inv @ dz))

        return e_x + e_z

    def h(self, mu, lms, z_ind): #Need to check if this works
        mu_temp = mu.reshape((3, int(mu.size/3)), order='F')
        lm = lms[:,z_ind.squeeze()].reshape((2,-1))
        ds = lm - mu_temp[0:2]
        r = np.sqrt(np.sum(ds*ds, axis=0))
        theta = np.arctan2(ds[1], ds[0]) - mu_temp[2]
        theta = unwrap(theta)
        z_hat = np.vstack((r,theta))

        return z_hat
