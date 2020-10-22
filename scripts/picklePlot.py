'''
Python script that plots SLAM estimate vs ground truth positions.
RMSE is also computed at the end.

This script loads up the estimated landmark list dumped by pickle.
By default the output file is saved at ~/.ros/map_outfile
'''

import os
import pickle
import pandas as pd
import matplotlib.pyplot as plt

# def plot_results():
class pickle_plot:
    def __init__(self):
        with open(os.path.expanduser('~/.ros/map_outfile'), 'rb') as fp:
            self.map = pickle.load(fp)

        script_dir = os.path.dirname(os.path.realpath(__file__))

        file_path = script_dir + "/small_track_truth" + "/blue_cone.csv"
        self.df_blue = pd.read_csv(file_path, header=None, usecols=[0, 1])

        file_path = script_dir + "/small_track_truth" + "/big_cone.csv"
        self.df_big = pd.read_csv(file_path, header=None, usecols=[0, 1])

        file_path = script_dir + "/small_track_truth" + "/yellow_cone.csv"
        self.df_yellow = pd.read_csv(file_path, header=None, usecols=[0, 1])



    def gen_plot(self):
        plt.figure(1)
        plt.cla()
        axes = plt.gca()

        self.df_blue.plot.scatter(  x=0, y=1, ax=axes, marker='o', facecolors='none', edgecolors='b', s=100, color='none')
        self.df_yellow.plot.scatter(x=0, y=1, ax=axes, marker='o', facecolors='none', edgecolors='b', s=100, color='none')
        self.df_big.plot.scatter(   x=0, y=1, ax=axes, marker='o', facecolors='none', edgecolors='b', s=100, color='none')

        print(self.df_blue)

        print(self.map)

        for lm in self.map:
            axes.scatter(lm[0], lm[1], marker='x', c='r')
        
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")

        plt.grid(True)

        script_dir = os.path.dirname(os.path.realpath(__file__))
        results_dir = os.path.join(script_dir, 'Results/')

        fileName = results_dir + "slam_comparison" + ".pdf"
        plt.savefig(fileName)
    
    def compute_rmse(self):
        '''
        Compute RMSE by looking for the closest pair?
        '''
        pass


if __name__ == "__main__":
    pp = pickle_plot()
    pp.gen_plot()