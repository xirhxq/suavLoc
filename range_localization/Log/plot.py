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
fig, axs = plt.subplots(2,2)
index_label = ['$P_x$', '$V_x$', '$P_y$', '$V_y$','$P_z$', '$V_z$','$B_z$']
file_label = ['IMU', 'Range', 'Pressure']
error_label = ['measure', 'predict', 'error']
axs[0,0].set_title('Translation')
axs[1,0].set_title('Velocity')
axs[0,1].set_title('ba')
axs[1,1].set_title('ba')
# datas = [predict, update1, update2]
datas = [predict, update1]
for i in range(len(datas)):
    data = datas[i]
    for j in range(3):
        axs[0,0].plot(data[:,0], data[:,j*2+1],'.-', label="{}_{}".format(file_label[i],index_label[j*2]))
        if loose_couple and i == 1:
            axs[0,0].plot(data[:,0], data[:,20+j],'.-', label="LQ_{}".format(index_label[j*2]))
        axs[1,0].plot(data[:,0], data[:,j*2+2],'.-', label="{}_{}".format(file_label[i],index_label[j*2+1]))
    axs[0,1].plot(data[:,0], data[:,7],'.-', label="{}_{}".format(file_label[i],index_label[6]))



plot_acc = True
if plot_acc:
    axs[0,1].plot(predict[:,0], predict[:,10],'.-', label="acc_z")
# index_label = ['x','y','z']
# for i in range(3):
#     axs[0,0].plot(true[:,0]*1e-9, true[:,i+1],'.-', label="True_{}".format(index_label[i]))



# q_enu0_b0 = Quaternion(0.821691564953,0.00507571770059,-0.00215338972064,0.569905757198)
# # q_b0_enu0 = q_enu0_b0.inverse
# velocity_enu = true_velocity[:,1:4]
# for i in range(velocity_enu.shape[0]):
#     velocity_enu[i,:] = q_enu0_b0.rotate(velocity_enu[i,:])

# xyz_label = ['x','y','z']
# for i in range(3):
#     axs[1,0].plot(predict[:,0], predict[:,i*2+2],'.-', label="{}_{}".format('est',index_label[i*2+1]))
#     axs[1,0].plot(true_velocity[:,0]*1e-9, velocity_enu[:,i],'.-', label="True_{}".format(xyz_label[i]))



# for i in range(2):
#     lns11 = axs[1,1].plot(update2[:,0] , update2[:,8+i],'.-', label="{}_{}".format(file_label[2],error_label[i]))
plot_truth = True
if plot_truth:
    height_turth = pd.read_csv("/media/ljn/Johny_SLAM/MBZIRC2023/Finals/experiments/dji_onboard/20230418/202304182224/height_above_takeoff.csv").to_numpy()
    lns12 = axs[1,1].plot((height_turth[:,0] - height_turth[0,0])/1e9 + update1[0,0], height_turth[:,1],'.-', label="truth")
    # lns13 = axs[1,1].plot(update1[:,0], update1[:,22],'.-', label="LQ_height")
    lns14 = axs[1,1].plot(update1[:,0], update1[:,5],'.-', label="fused_height")
    
    t_true = (height_turth[:,0] - height_turth[0,0])/1e9 + update1[0,0]
    z_true = height_turth[:,1]
    error_t = update1[:,0]

    true_z = np.interp(error_t, t_true, z_true) 
    error_z = true_z - update1[:,5]
    z_mean = error_z.mean()
    z_se   = error_z.std() / np.sqrt(error_z.shape[0])
    z_err  = error_z - z_mean
    print("mean: ",z_mean,"Standard Deviation: ",z_se)
    lns15 = axs[1,1].plot(update1[:,0], z_err,'.-', label="error")



    # ax2 = axs[1,1].twinx()
    # lns21 = ax2.plot(update2[:,0] , update2[:,17],'.-', label="R_z")
    # lns22 = ax2.plot(update2[:,0] , update2[:,18],'.-', label="DOP")
    # lns23 = ax2.plot(update2[:,0] , 100/((update2[:,18]/2)**0.5),'.-', label="designed_DOP")
    # ax2.axhline(2)
    # ax2.set_ylabel('R_z')
    # ax2.set_xlabel('Same')

    # lns = lns11 + lns12 + lns13 + lns14 + lns21 + lns22 + lns22
    # labs = [l.get_label() for l in lns]
    # axs[1,1].legend(lns, labs)
    # plt.show()


for j in range(4):
    # axs[j].set_xlim(386,389)
    axs[j%2, int(j/2)].grid()
    axs[j%2, int(j/2)].legend()






plt.grid()
    ######for ikfom#######
# else:
#     #######for normal#######
#     fig, axs = plt.subplots(3,2)
#     lab_pre = ['', 'pre-x', 'pre-y', 'pre-z']
#     lab_out = ['', 'out-x', 'out-y', 'out-z']
#     plot_ind = range(7,10)
#     time=a_pre[:,0]
#     axs[0,0].set_title('Attitude')
#     axs[1,0].set_title('Translation')
#     axs[2,0].set_title('Velocity')
#     axs[0,1].set_title('bg')
#     axs[1,1].set_title('ba')
#     axs[2,1].set_title('Gravity')
#     for i in range(1,4):
#         for j in range(6):
#             axs[j%3, j/3].plot(time, a_pre[:,i+j*3],'.-', label=lab_pre[i])
#             axs[j%3, j/3].plot(time, a_out[:,i+j*3],'.-', label=lab_out[i])
#     for j in range(6):
#         # axs[j].set_xlim(386,389)
#         axs[j%3, j/3].grid()
#         axs[j%3, j/3].legend()
#     plt.grid()
#     #######for normal#######


# #### Draw IMU data
# fig, axs = plt.subplots(2)
# imu=np.loadtxt('imu.txt')
# time=imu[:,0]
# axs[0].set_title('Gyroscope')
# axs[1].set_title('Accelerameter')
# lab_1 = ['gyr-x', 'gyr-y', 'gyr-z']
# lab_2 = ['acc-x', 'acc-y', 'acc-z']
# for i in range(3):
#     # if i==1:
#     axs[0].plot(time, imu[:,i+1],'.-', label=lab_1[i])
#     axs[1].plot(time, imu[:,i+4],'.-', label=lab_2[i])
# for i in range(2):
#     # axs[i].set_xlim(386,389)
#     axs[i].grid()
#     axs[i].legend()
# plt.grid()

# #### Draw time calculation
# plt.figure(3)
# fig = plt.figure()
# font1 = {'family' : 'Times New Roman',
# 'weight' : 'normal',
# 'size'   : 12,
# }
# c="red"
# a_out1=np.loadtxt('Log/mat_out_time_indoor1.txt')
# a_out2=np.loadtxt('Log/mat_out_time_indoor2.txt')
# a_out3=np.loadtxt('Log/mat_out_time_outdoor.txt')
# # n = a_out[:,1].size
# # time_mean = a_out[:,1].mean()
# # time_se   = a_out[:,1].std() / np.sqrt(n)
# # time_err  = a_out[:,1] - time_mean
# # feat_mean = a_out[:,2].mean()
# # feat_err  = a_out[:,2] - feat_mean
# # feat_se   = a_out[:,2].std() / np.sqrt(n)
# ax1 = fig.add_subplot(111)
# ax1.set_ylabel('Effective Feature Numbers',font1)
# ax1.boxplot(a_out1[:,2], showfliers=False, positions=[0.9])
# ax1.boxplot(a_out2[:,2], showfliers=False, positions=[1.9])
# ax1.boxplot(a_out3[:,2], showfliers=False, positions=[2.9])
# ax1.set_ylim([0, 3000])

# ax2 = ax1.twinx()
# ax2.spines['right'].set_color('red')
# ax2.set_ylabel('Compute Time (ms)',font1)
# ax2.yaxis.label.set_color('red')
# ax2.tick_params(axis='y', colors='red')
# ax2.boxplot(a_out1[:,1]*1000, showfliers=False, positions=[1.1],boxprops=dict(color=c),capprops=dict(color=c),whiskerprops=dict(color=c))
# ax2.boxplot(a_out2[:,1]*1000, showfliers=False, positions=[2.1],boxprops=dict(color=c),capprops=dict(color=c),whiskerprops=dict(color=c))
# ax2.boxplot(a_out3[:,1]*1000, showfliers=False, positions=[3.1],boxprops=dict(color=c),capprops=dict(color=c),whiskerprops=dict(color=c))
# ax2.set_xlim([0.5, 3.5])
# ax2.set_ylim([0, 100])

# plt.xticks([1,2,3], ('Outdoor Scene', 'Indoor Scene 1', 'Indoor Scene 2'))
# # # print(time_se)
# # # print(a_out3[:,2])
# plt.grid()
# plt.savefig("time.pdf", dpi=1200)
plt.show()
# plt.savefig("result_plot.png", dpi=1200)
