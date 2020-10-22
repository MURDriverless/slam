'''
Python script that plots SLAM estimate vs ground truth positions.
RMSE is also computed at the end.

This script loads up the estimated landmark list dumped by pickle.
By default the output file is saved at ~/.ros/map_outfile

Usage: python3 picklePlot.py
'''

import os
import pickle
import pandas as pd
import matplotlib.pyplot as plt
import math

# def plot_results():
class pickle_plot:
    def __init__(self):

        # load previous map data from map_outfile
        with open(os.path.expanduser('~/.ros/map_outfile'), 'rb') as fp:
            self.map = pickle.load(fp)

        script_dir = os.path.dirname(os.path.realpath(__file__))

        file_path = script_dir + "/small_track_truth" + "/blue_cone.csv"
        self.df_blue = pd.read_csv(file_path, header=None, usecols=[0, 1])

        file_path = script_dir + "/small_track_truth" + "/big_cone.csv"
        self.df_big = pd.read_csv(file_path, header=None, usecols=[0, 1])

        file_path = script_dir + "/small_track_truth" + "/yellow_cone.csv"
        self.df_yellow = pd.read_csv(file_path, header=None, usecols=[0, 1])

        self.df_all = pd.concat([self.df_blue, self.df_big, self.df_yellow]).reset_index(drop=True)


    def gen_plot(self):
        plt.figure(1)
        plt.cla()
        axes = plt.gca()

        self.df_blue.plot.scatter(  x=0, y=1, ax=axes, marker='o', facecolors='none', edgecolors='b', s=100, color='none')
        self.df_yellow.plot.scatter(x=0, y=1, ax=axes, marker='o', facecolors='none', edgecolors='b', s=100, color='none')
        self.df_big.plot.scatter(   x=0, y=1, ax=axes, marker='o', facecolors='none', edgecolors='b', s=100, color='none')

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
        Computes RMSE by matching estimated landmark with closest ground
        truth
        '''

        # gnd_truth holds a list of tuple [(x1,y1), ... ,(xn,yn)]
        gnd_truth = [tuple(x) for x in self.df_all.to_numpy()]

        rmse_sum = 0
        
        for lm in self.map:
            min_dist = float('inf')
            chosen_gt = (0,0)

            # go through each ground truth and find the closest one
            for gt in gnd_truth:
                dist = self.euclidean_dist(lm, gt)
                if dist < min_dist:
                    chosen_gt = gt
                    min_dist = dist
            
            # Check matching pair
            print(lm, chosen_gt)

            rmse_sum += ( (lm[0] - chosen_gt[0])**2 + (lm[1] - chosen_gt[1])**2 )
        
        n = len(gnd_truth)
        rmse = math.sqrt(rmse_sum / n)

        print("The RMSE is ", rmse, " (m)")
            
    def euclidean_dist(self, tuple1, tuple2):
        '''
        Computes euclidean distance between two tuples
        (x1, y1) and (x2, y2)
        '''
        (x1, y1) = tuple1
        (x2, y2) = tuple2
        return math.sqrt( (y2-y1)**2 + (x2-x1)**2 )

if __name__ == "__main__":
    pp = pickle_plot()
    pp.gen_plot()
    pp.compute_rmse()