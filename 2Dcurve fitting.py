from numpy import array
import scipy.io as sio
from scipy.optimize import leastsq

def __residual(params, y, a, b, c):
    p0, e0, p1, e1 = params
    return p0 * a ** e0 + p1 * b ** e1 - y


if __name__ == "__main__":
    # load a, b, c
    # guess initial values for p0, e0, p1, e1
    mat = sio.loadmat('omni_radius_2d_rp_swept.mat')
    print(mat['roll'])
    # p_opt = leastsq(__residual,  array([p0, e0, p1, e1]), args=(y, a, b, c))
    # print('y = %f a^%f + %f b^%f %f c^%f',map(float, p_opt))
