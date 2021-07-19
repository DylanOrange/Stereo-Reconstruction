import numpy as np
from scipy.spatial.transform import Rotation as R, rotation

T_02 = np.array([5.956621e-02, 2.900141e-04, 2.577209e-03])
T_03 = np.array([-4.731050e-01, 5.551470e-03, -5.250882e-03])

T23 = -T_02 + T_03

print('T23', T23)

R_02 = np.array([[9.999758e-01, -5.267463e-03, -4.552439e-03], 
                [5.251945e-03, 9.999804e-01, -3.413835e-03], 
                [4.570332e-03, 3.389843e-03, 9.999838e-01]])

R_03 = np.array([[9.995599e-01, 1.699522e-02, -2.431313e-02], 
                [-1.704422e-02, 9.998531e-01, -1.809756e-03], 
                [2.427880e-02, 2.223358e-03, 9.997028e-01]])

R_23 = R_02.T.dot(R_03)

print('R_23', R_23)

r = R.from_matrix(R_23)

r_euler_gt = r.as_euler('zyx', degrees=True)
print('r_euler_gt', r_euler_gt)

r_eulers = []
ts = []

f = open("./rt_orb_40_ba.txt")
line = f.readline()
while line:
    str_list = line.split(" ")
    if(len(str_list) == 9):
        _R = np.zeros((3,3))
        for i in range(3):
            for j in range(3):
                _R[i, j] = str_list[i*3+j]
        r = R.from_matrix(_R)
        r_euler = r.as_euler('zyx', degrees=True)
        r_eulers.append(r_euler)
    elif(len(str_list) == 3):
        _t = np.zeros(3)
        for i in range(3):
            _t[i] = str_list[i]
        ts.append(_t)        
    line = f.readline()
f.close()

r_euler_error = []
print("r_eulers: ")
for r_euler in r_eulers:
    # print(r_euler)
    r_euler_error.append(abs(r_euler - r_euler_gt))

r_euler_error = np.array(r_euler_error)
r_euler_error = np.mean(r_euler_error, axis=0)
print(r_euler_error)

t_error = []
print("ts: ")
for t in ts:
    t_error.append(np.linalg.norm(t - T23))

t_error = np.array(t_error)
t_error = np.mean(t_error)
print(t_error)