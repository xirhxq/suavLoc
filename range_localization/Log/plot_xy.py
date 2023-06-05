# import matplotlib
# matplotlib.use('Agg')
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
# from pyquaternion import Quaternion

loose_couple = True
# predict=pd.read_csv('src/range_localization/Log/predict.txt')
# predict=predict.dropna().to_numpy()
# print(predict.shape)
predict=np.loadtxt('src/range_localization/Log/predict.txt')
print(predict.shape)
# update1=pd.read_csv('src/range_localization/Log/update1.txt')
# update1=update1.dropna().to_numpy()

# update2=pd.read_csv('src/range_localization/Log/update2.txt')
# update2=update2.dropna().to_numpy()
update1=np.loadtxt('src/range_localization/Log/update1.txt')
update2=np.loadtxt('src/range_localization/Log/update2.txt')

# true=pd.read_csv("/media/ljn/BIT_SLAM/MBZIRC2023/Finals/experiments/RTK_UWB_Compare/rtk_20230228/path1/rtk_trajectory_enu.csv")
# true=true.to_numpy()

# true_velocity=pd.read_csv("src/range_localization/test/velocity.csv",usecols=['field.header.stamp','field.vector.x','field.vector.y','field.vector.z'])
# true_velocity=true_velocity.to_numpy()

# delta_t = predict[0,0]-true[0,0]*1e-9
######for ikfom
fig1 = plt.plot(update1[:,1],update1[:,3])


plt.grid()
plt.savefig("src/range_localization/Log/result_xy.png", dpi=1200)
plt.clf()

fig2 = plt.plot(update1[:,0],update1[:,5])
plt.grid()
plt.savefig("src/range_localization/Log/result_z.png", dpi=1200)
