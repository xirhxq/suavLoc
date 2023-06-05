import numpy as np
import math
import sys
import pandas as pd
import os
import pymap3d as pm
import platform
import matplotlib.pyplot as plt
import scienceplots







if __name__ == '__main__':

    predict=np.loadtxt('src/range_localization/Log/predict.txt')
    # print(predict.shape)

    true=pd.read_csv("src/range_localization/test/acceleration_ground.csv",usecols=['field.header.stamp',
                                'field.vector.x','field.vector.y','field.vector.z'])
    true=true.to_numpy()
    with plt.style.context(['science','ieee','no-latex']):
    # with plt.style.context(['high-vis','no-latex']):
        
        cm = 1/2.54 # centimeters in inches
        loop_label = ['x', 'y', 'z']
        loop_color = ['r', 'g', 'b']
        fig,ax = plt.subplots(figsize=(8.5*cm,6*cm),dpi = 300)
        for i in range(3):
            ax.plot(true[:,0]/1e9, true[:,i+1], '{}-'.format(loop_color[i]),label = r'True-$a_{}$'.format(loop_label[i]))
            ax.plot(predict[:,0], predict[:,i+8], '{}-.'.format(loop_color[i]), label = r'Est-$a_{}$'.format(loop_label[i]))
            # ax.plot(true[:,0]/1e9, true[:,i+1],label = r'True-$a_{}$'.format(loop_label[i]))
            # ax.plot(predict[:,0], predict[:,i+8], label = r'Est-$a_{}$'.format(loop_label[i]))
        ax.set_ylabel(r'acceleration (m/s^2)')
        ax.set_xlabel(r'Time (s)')
        plt.legend()
        plt.show()
        # plt.savefig(outputPath + "Trajectory{}.png".format(i), dpi=300)