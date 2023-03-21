import numpy as np
np.set_printoptions(precision=6, linewidth=65536, suppress=True, threshold=np.inf)
import math
from scipy import sparse
from scipy.spatial import KDTree
import osqp

class MPCController():
    def __init__(self, L, dt):
        self.L = L

        self.Nx = 3
        self.Nu = 2

        self.Nc = 30
        self.Np = 60

        self.T = dt

        self.pre_u = np.zeros(self.Nu)



    def set_global_path(self, global_path):
        self.global_path = global_path

    def set_ref_traj(self, ref_traj):
        self.tree = KDTree(ref_traj[:, :2])



    def select_action(self, state, ref_traj, time_step):
        u_cur, delta_u_cur = self.Solve(state, ref_traj, time_step)
        u_ref = ref_traj[time_step][-2:]
        control = u_ref + u_cur
        print(f'[mpc] step {time_step}: control {control}')
        return np.expand_dims(control, axis=0)


    def get_reference_x(self, state, ref_traj, t):
        x = state.pose

        ### path ref
        nearest_ref_info = self.tree.query(x[:2])
        nearest_ref_x = ref_traj[nearest_ref_info[1]]

        ### traj ref
        # nearest_ref_x = ref_traj[t]
        return nearest_ref_x

    def Solve(self, state, ref_traj, t=None):
        x = state.pose
        u_pre = self.pre_u

        nearest_ref_x = self.get_reference_x(state, ref_traj, t)


        a = np.array([
            [1.0,  0,   -nearest_ref_x[3] * math.sin(nearest_ref_x[2]) * self.T],
            [0,   1.0,   nearest_ref_x[3] * math.cos(nearest_ref_x[2]) * self.T],
            [0,   0,     1]
        ])

        b = np.array([
            [math.cos(nearest_ref_x[2]) * self.T,             0],
            [math.sin(nearest_ref_x[2]) * self.T,             0],
            [math.tan(nearest_ref_x[4]) * self.T / self.L,     nearest_ref_x[3] * self.T / (self.L * math.pow(math.cos(nearest_ref_x[4]), 2.0))]
        ])

        A = np.zeros([self.Nx + self.Nu, self.Nx + self.Nu])
        A[0 : self.Nx, 0 : self.Nx] = a
        A[0 : self.Nx, self.Nx : ] =  b
        A[self.Nx :, self.Nx :] = np.eye(self.Nu)

        B = np.zeros([self.Nx + self.Nu, self.Nu])
        B[0 : self.Nx, :] = b
        B[self.Nx :, : ] = np.eye(self.Nu)

        C = np.array([[1, 0, 0, 0, 0], [0, 1, 0 ,0 ,0], [0, 0, 1, 0, 0]])

        theta = np.zeros([self.Np * self.Nx, self.Nc * self.Nu])
        phi = np.zeros([self.Np * self.Nx, self.Nu + self.Nx])
        tmp = C

        for i in range(1, self.Np + 1):
            phi[self.Nx * (i - 1) : self.Nx * i] = np.dot(tmp, A)

            tmp_c = np.zeros([self.Nx, self.Nc * self.Nu])
            tmp_c[ :, 0 : self.Nu] = np.dot(tmp, B)

            if i > 1:
                tmp_c[ :, self.Nu :] = theta[self.Nx * (i - 2) : self.Nx * (i - 1), 0 : -self.Nu]

            theta[self.Nx * (i - 1) : self.Nx * i, :] = tmp_c

            tmp = np.dot(tmp, A)


        Q = np.eye(self.Nx * self.Np)

        R = 0.2 * np.eye(self.Nu * self.Nc)

        rho = 10

        H = np.zeros((self.Nu * self.Nc + 1, self.Nu * self.Nc + 1))
        H[0 : self.Nu * self.Nc, 0 : self.Nu * self.Nc] = np.dot(np.dot(theta.transpose(), Q), theta) + R
        H[-1 : -1] = rho

        kesi = np.zeros((self.Nx + self.Nu, 1))
        diff_x = x - nearest_ref_x[:3]
        diff_x = diff_x.reshape(-1, 1)
        kesi[: self.Nx, :] = diff_x
        diff_u = u_pre.reshape(-1, 1)
        kesi[self.Nx :, :] = diff_u

        F = np.zeros((1, self.Nu * self.Nc + 1))
        F_1 = 2 * np.dot(np.dot(np.dot(phi, kesi).transpose(), Q), theta)
        F[ 0,  0 : self.Nu * self.Nc] = F_1

        # constraints
        umax = np.array([[0.2], [np.deg2rad(45)]])
        umin = -umax

        delta_umax = np.array([[0.1], [0.08]])
        delta_umin = -delta_umax

        A_t = np.zeros((self.Nc, self.Nc))
        for row in range(self.Nc):
            for col in range(self.Nc):
                if row >= col:
                    A_t[row, col] = 1.0


        A_I = np.kron(A_t, np.eye(self.Nu))

        A_cons = np.zeros((self.Nc * self.Nu, self.Nc * self.Nu + 1))
        A_cons[0 : self.Nc * self.Nu, 0 : self.Nc * self.Nu] = A_I

        U_t = np.kron(np.ones((self.Nc, 1)), u_pre.reshape(-1, 1))

        U_min = np.kron(np.ones((self.Nc, 1)), umin)
        U_max = np.kron(np.ones((self.Nc, 1)), umax)

        LB = U_min - U_t
        UB = U_max - U_t

        delta_Umin = np.kron(np.ones((self.Nc, 1)), delta_umin)
        delta_Umax = np.kron(np.ones((self.Nc, 1)), delta_umax)

        delta_Umin = np.vstack((delta_Umin, [0]))
        delta_Umax = np.vstack((delta_Umax, [10]))

        A_1_cons = np.eye(self.Nc * self.Nu + 1, self.Nc * self.Nu + 1)

        A_cons = np.vstack((A_cons, A_1_cons))

        LB = np.vstack((LB, delta_Umin))
        UB = np.vstack((UB, delta_Umax))

        # Create an OSQP object
        prob = osqp.OSQP()
    
        H = sparse.csc_matrix(H)
        A_cons = sparse.csc_matrix(A_cons)

        # Setup workspace
        prob.setup(H, F.transpose(), A_cons, LB, UB, verbose=False)
        # prob.update_settings(verbose=False)
        # import pdb; pdb.set_trace()

        res = prob.solve()

        # Check solver status
        if res.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')

        u_cur = u_pre + res.x[0 : self.Nu]
        self.pre_u = u_cur

        return u_cur, res.x[0 : self.Nu]

