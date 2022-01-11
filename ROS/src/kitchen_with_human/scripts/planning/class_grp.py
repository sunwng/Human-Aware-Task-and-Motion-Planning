import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import pdist, squareform, cdist

class grp(object):
    def __init__(self, _name):
        self.name = _name
        # BY DEFAULT, DONT DO EPSILON RUN-UP
        self.DO_EPSRU = False
        print ("[%s] GAUSSIAN RANDOM PATH" % (self.name))

    """
        SET ANCHOR POINTS & HYPERPARAMETERS & VELOCITY
    """
    def set_data(self, _px, _hyp_mean, _hyp_var, _DO_EPSRU=False, _teps=0.01):
        # SET POSITIONS (ANCHORS) TO FOLLOW
        self.px = _px
        # NUMBER OF ANCHORS
        self.nx = self.px.shape[0]
        # DIMENSION OF POSITION
        self.dim = self.px.shape[1]
        # SET HYPERPARAMETERS (FOR BOTH MEAN AND VAR)
        self.hyp_mean  = _hyp_mean
        self.hyp_var   = _hyp_var
        # SET TIME INDICES
        self._get_timeidx()
        # EPSILON RUN-UP
        if _DO_EPSRU:
            self._do_epsrunup(_teps)

    """
        KERNEL FUNCTION
    """
    def kernel_se(self, _X, _Y, _hyp={'gain':1, 'len':1}):
        hyp_gain = float(_hyp['gain'])
        hyp_len  = 1/float(_hyp['len'])
        pairwise_dists = cdist(_X, _Y, 'euclidean')
        K = hyp_gain*np.exp(-pairwise_dists ** 2 / hyp_len)
        return K

    """
        COMPUTE TIME INDICES FOR SELF.PX
            (CALLED INSIDE 'SET_DATA')
    """
    def _get_timeidx(self):
        # TIME INDICES FOR TRAINING ANCHOR POINTS
        self.tx  = np.zeros((self.nx, 1))
        self.sum_dist = 0.0
        self.max_dist = 0.0
        for i in range(self.nx-1):
            prevx    = self.px[i, :]
            currx    = self.px[i+1, :]
            dist     = np.linalg.norm(currx-prevx)
            self.tx[i+1]  = self.tx[i]+dist
            # COMPUTE THE TOTAL DISTANCE
            self.sum_dist = self.sum_dist + dist
            # COMPUTE THE MAXIMUM DISTANCE BETWEEN POINTS
            if dist > self.max_dist:
                self.max_dist = dist

        # NORMALIZE TIME INDICES TO BE WITHIN 0~1
        self.tx = self.tx / self.tx[-1]

    """
        ADD EPSILON RUN-UP TO BOTH 'TX_RU' AND 'PX_RU'
            (CALLED INSIDE 'SET_DATA')
    """
    def _do_epsrunup(self, _teps=0.01):
        # NOW WE ARE DOING EPSILON RUN-UP
        self.DO_EPSRU   = True
        self.teps       = _teps
        self.tx_ru = np.insert(self.tx, 1, self.teps, 0)
        self.tx_ru = np.insert(self.tx_ru, -1, 1-self.teps, 0)

        # EPS-RUNUP OF START
        diff = (self.px[1,:]-self.px[0,:])
        uvec = diff / np.linalg.norm(diff)
        peru = self.px[0,:] + uvec*self.sum_dist*_teps
        self.px_ru = np.insert(self.px, 1, peru, 0)

        # EPS-RUNUP OF END
        diff = (self.px[-1,:]-self.px[-2,:])
        uvec = diff / np.linalg.norm(diff)
        peru = self.px[-1,:] - uvec*self.sum_dist*_teps
        self.px_ru = np.insert(self.px_ru, -1, peru, 0)

    """
        COMPUTE MEAN AND VARAINCE PATHS
    """
    def compute_grp(self, _vel):
        # NUMBER OF POINTS
        self.nz = (int)(self.sum_dist/_vel)
        # TIME INDICES FOR INTERPOLATED PATHS (Z)
        self.tz = np.linspace(0, 1, self.nz).reshape(-1, 1)
        # GET TIME INDICES AND TRAJECTORY FOR TRAINING
        if self.DO_EPSRU:
            self.tx_used = self.tx_ru
            self.px_used = self.px_ru
        else:
            self.tx_used = self.tx
            self.px_used = self.px
        # COMPUTE MEAN PATH
        k_zz = self.kernel_se(self.tz, self.tz, self.hyp_mean)
        self.k_zx_mean = self.kernel_se(self.tz, self.tx_used, self.hyp_mean)
        self.k_xx_mean = self.kernel_se(self.tx_used, self.tx_used, self.hyp_mean)\
                + self.hyp_mean['noise']*np.eye(self.tx_used.shape[0])
        px_used_mean = self.px_used.mean(axis=0)
        self.muz = np.matmul(self.k_zx_mean,
                np.linalg.solve(self.k_xx_mean, self.px_used-px_used_mean))+px_used_mean

        # COMPUATE VARAINCE


        print ("[%s] THE LENGTH OF A INTERPOLATED TRAJ IS [%d]" % (self.name, self.nz))


class hgrp(object):

    def __init__(self, _name='HGRP', _VERBOSE=True):
        self.name    = _name
        self.VERBOSE = _VERBOSE
        if self.VERBOSE:
            print ("[%s] INSTANTIATED" % (self.name))

    def set_x_anchor(self, _x_anchor,_SAMEINTV=False,_EPSRU=False,_teps=0.01):
        self.x_anchor = _x_anchor
        self.n_anchor = self.x_anchor.shape[0]
        self.dim      = self.x_anchor.shape[1]
        # SET TIME INDICES
        self.set_t_anchor(_SAMEINTV=_SAMEINTV,_EPSRU=_EPSRU,_teps=_teps)

    def set_t_anchor(self,_SAMEINTV=False,_EPSRU=False,_teps=0.01):
        if _SAMEINTV:
            """ USE SAME INTERVAL """
            self.t_anchor = np.linspace(0, 1, num=self.n_anchor).reshape((-1, 1))
            self.sum_dist = 0.0
            for i in range(self.n_anchor-1):
                prevx    = self.x_anchor[i, :]
                currx    = self.x_anchor[i+1, :]
                dist     = np.linalg.norm(currx-prevx)
                self.sum_dist = self.sum_dist + dist
        else:
            """ SET TIME INVERVAL BASED ON DISTANCE """
            self.t_anchor = np.zeros((self.n_anchor, 1))
            self.sum_dist = 0.0
            self.max_dist = 0.0
            for i in range(self.n_anchor-1):
                prevx    = self.x_anchor[i, :]
                currx    = self.x_anchor[i+1, :]
                dist     = np.linalg.norm(currx-prevx)
                self.t_anchor[i+1] = self.t_anchor[i]+dist
                """ COMPUTE THE TOTAL DISTANCE """
                self.sum_dist = self.sum_dist + dist
                """ COMPUTE THE MAXIMUM DISTANCE BETWEEN POINTS """
                if dist > self.max_dist:
                    self.max_dist = dist
            self.t_anchor = self.t_anchor / self.t_anchor[-1] # np.linspace(0, dist/10, num=self.n_anchor).reshape((-1, 1))
        if _EPSRU:
            """ EPSILON RUN-UP """
            self.teps_anchor = _teps
            self.t_anchor = np.insert(self.t_anchor, 1, self.teps_anchor, 0)
            self.t_anchor = np.insert(self.t_anchor, -1, 1-self.teps_anchor, 0)
            """ FOR START POINT """
            diff = (self.x_anchor[1,:]-self.x_anchor[0,:])
            if np.linalg.norm(diff) == 0:
                uvec = 0.0*diff
            else:
                uvec = diff / np.linalg.norm(diff)
            peru = self.x_anchor[0,:] + uvec*self.sum_dist*self.teps_anchor
            self.x_anchor = np.insert(self.x_anchor, 1, peru, 0)
            """ FOR FINAL POINT """
            diff = (self.x_anchor[-1,:]-self.x_anchor[-2,:])
            if np.linalg.norm(diff) == 0:
                uvec = 0.0*diff
            else:
                uvec = diff / np.linalg.norm(diff)
            peru = self.x_anchor[-1,:] - uvec*self.sum_dist*self.teps_anchor
            self.x_anchor = np.insert(self.x_anchor, -1, peru, 0)

    def compute_grp(self,_len=100,_hyp_mean={'gain':1,'len':1,'noise':1e-8},
                    _hyp_var={'gain':1,'len':1,'noise':1e-8},
                    _hyp_hvar={'gain':1,'len':1,'noise':1e-8}):
        self.hyp_mean = _hyp_mean
        """ TEST TIME INDICES """
        self.len    = (int)(_len)
        self.ntest  = (int)(_len)
        self.t_test = np.linspace(0, 1, self.len).reshape((-1,1))
        """ COMPUTE MEAN PATHS """
        self.hyp_mean = _hyp_mean
        self.k_test_anchor_mean = self.kernel_se(self.t_test, self.t_anchor, self.hyp_mean)
        self.k_anchor_mean = self.kernel_se(self.t_anchor, self.t_anchor, self.hyp_mean)\
                + self.hyp_mean['gain']*self.hyp_mean['noise']*np.eye(self.t_anchor.shape[0])
        x_anchor_mean = self.x_anchor.mean(axis=0)
        self.mu_test  = np.matmul(self.k_test_anchor_mean,
                np.linalg.solve(self.k_anchor_mean, self.x_anchor-x_anchor_mean))+x_anchor_mean
        """ COMPUTE VARAIANCE PATHS """
        self.hyp_var = _hyp_var
        self.k_test_var = self.kernel_se(self.t_test, self.t_test, self.hyp_var)\
                + self.hyp_var['gain']*self.hyp_var['noise']*np.eye(self.t_test.shape[0])
        self.k_test_anchor_var = self.kernel_se(self.t_test, self.t_anchor, self.hyp_var)
        self.k_anchor_var = self.kernel_se(self.t_anchor, self.t_anchor, self.hyp_var)\
                + self.hyp_var['gain']*self.hyp_var['noise']*np.eye(self.t_anchor.shape[0])
        self.var_test = self.k_test_var - np.matmul(self.k_test_anchor_var,
                np.linalg.solve(self.k_anchor_var, self.k_test_anchor_var.T))
        """ COMPUTE (HETEROSCEDASTIC) VARAINCE """
        self.hyp_hvar = _hyp_hvar
        self.k_test_hvar = self.kernel_se(self.t_test, self.t_test, self.hyp_hvar)\
                + self.hyp_hvar['gain']*self.hyp_hvar['noise']*np.eye(self.t_test.shape[0])
        tempidx = [0,1,-2,-1]
        self.t_hvar = self.t_anchor[tempidx,:]
        self.k_test_anchor_hvar = self.kernel_se(self.t_test, self.t_hvar, self.hyp_hvar)
        self.k_anchor_hvar = self.kernel_se(self.t_hvar, self.t_hvar, self.hyp_hvar)\
                + self.hyp_hvar['gain']*self.hyp_hvar['noise']*np.eye(self.t_hvar.shape[0])
        self.hvar_test = self.k_test_hvar - np.matmul(self.k_test_anchor_hvar,
                np.linalg.solve(self.k_anchor_hvar, self.k_test_anchor_hvar.T))
        self.var_total_test = self.var_test + self.hvar_test
        self.var_total_test_vec = np.diag(self.var_total_test).reshape((-1,1))

        """ CHOLESKY DECOMPOSITION """
        self.var_total_test_chol = np.linalg.cholesky(self.var_total_test)

    def get_meanpaths(self):
        return self.mu_test

    def get_samplepaths(self,_npath=1):
        self.npath = _npath
        samplepaths = []
        for i in range(self.npath):
            """ FOR EACH PATH """
            R = np.random.randn(self.ntest,self.dim)
            samplepath = self.mu_test+np.matmul(self.var_total_test_chol,R)
            samplepaths.append(samplepath)
        return samplepaths, self.mu_test

    def kernel_se(self, _X, _Y, _hyp={'gain':1,'len':1,'noise':1e-8}):
        hyp_gain = float(_hyp['gain'])**2
        hyp_len  = 1/float(_hyp['len'])
        if len(_X.shape)<=1: _X=_X.reshape((-1,1))
        if len(_Y.shape)<=1: _Y=_Y.reshape((-1,1))
        pairwise_dists = cdist(_X, _Y, 'euclidean')
        K = hyp_gain*np.exp(-pairwise_dists ** 2 / (hyp_len**2))
        return K


def plot_sampletraj(y_samplepahts, meanpath, ntraj):
    x_samples = np.linspace(-2, 2, int(ntraj)).reshape((-1,1))
    
    plt.figure(figsize=(10,4))
    for y_samples in y_samplepahts:
        plt.plot(x_samples, y_samples)
    plt.title("Sampeld Trajectory")
    # plt.ylim(0, 1)
    # plt.xlim(-2,2)
    plt.scatter(x_samples, meanpath)
    plt.plot(x_samples, meanpath)
    plt.show()

def plot_bestquery(meanpath, ntraj):
    x_samples = np.linspace(-0.5, 0.5, int(ntraj)).reshape((-1,1))
    np.savez("save/best_trajectory.npz", x=x_samples, y=meanpath)
    print("SAVED. Best Trajectory")
    plt.figure(figsize=(10,4))
    plt.title("Max Reward Query")
    plt.ylim(0, 1)
    plt.xlim(-0.5,0.5)
    plt.ylabel("Z axis")
    plt.xlabel("Y axis")
    plt.plot(x_samples, meanpath,'g-*',label="Trajectory", lw=5)
    plt.legend(loc="lower right")
    plt.show()   

def plot_query(querypath1, queryx1, querypath2, queryx2, i):
    x_samples1 = np.linspace(-0.5, 0.5, int(queryx1)).reshape((-1,1))
    x_samples2 = np.linspace(-0.5, 0.5, int(queryx2)).reshape((-1,1))

    plt.figure(figsize=(10,4))
    # for y_samples in y_samplepahts:
    #     plt.plot(x_samples, y_samples)
    plt.title("#{}. Sampeld Trajectory".format(i+1), size=20)
    plt.ylim(0, 1)
    plt.xlim(-0.5,0.5)
    plt.ylabel("Z axis", size=15)
    plt.xlabel("Y axis", size=15)
    plt.plot(x_samples1, querypath1,'r-D',label=r'$\xi_{a}$', markersize=7)
    plt.plot(x_samples2, querypath2, 'b--o',label=r'$\xi_{b}$', markersize=7)
    plt.legend(loc="upper right", fontsize=15)
    plt.show()

def plot_sampletrajs(meanpath, ntraj, idxs, anchors):
    x_samples = np.linspace(0, ntraj, int(ntraj)).reshape((-1,1))
    anchor_x_samples = np.linspace(0, ntraj, int(len(anchors))).reshape((-1,1))
    plt.figure(figsize=(10,4))
    plt.title("Joint{}:Mean Path; len=0.1".format(idxs), size=20)
    plt.ylim(3.14, -3.14)
    plt.ylabel("Joint value",size=15)
    plt.xlabel("Step",size=15)

    plt.xlim(0,ntraj)
    plt.scatter(anchor_x_samples, anchors, c= 'g', s=30, label="Plan Step")
    plt.plot(x_samples, meanpath, 'r',label="Mean Path", lw="3")
    plt.legend(loc="upper right")
    plt.show()

if __name__ == "__main__":
    _lentraj=100
    _hyp_mean={'gain':0.2, 'len':4, "noise":1e-9}
    _hyp_var={'gain':0.2, 'len':4, 'noise':1e-9}    
    _hyp_hvar={'gain':0.2, 'len':4, 'noise':1e-9}
    H = hgrp()
    H.set_x_anchor(_x_anchor=np.array([[0],[1.5],[0]]).reshape(-1,1), _SAMEINTV=True, _EPSRU=True, _teps=0.01) #np.zeros(2).reshape(-1,1)
    H.compute_grp(_len=_lentraj,_hyp_mean=_hyp_mean,_hyp_var=_hyp_var,_hyp_hvar=_hyp_hvar)
    randompaths, meanpath = H.get_samplepaths(_npath=20)
    plot_sampletraj(randompaths, meanpath, _lentraj)
















