import os
import joblib
import numpy as np
import pandas as pd
import termcolor
import time
from colorama import Fore, Back, Style

absPath = os.path.dirname(__file__)
relPath = "dependencies"
filesPath = os.path.join(absPath,relPath)


def pca_model(filename='pca_model.pkl',path=filesPath):
    os.chdir(filesPath)
    pca = joblib.load(filename)
    os.chdir(absPath)
    return pca

def scaler_model(filename='scaler_model.pkl',path=filesPath):
    os.chdir(filesPath)
    scaler = joblib.load(filename)
    os.chdir(absPath)
    return scaler

def tree_model(filename='tree_pca.pkl',path=filesPath):
    os.chdir(filesPath)
    tree = joblib.load(filename)
    os.chdir(absPath)
    return tree


def checkAnomaly(uav, uav_stat, pca, scaler, tree, time_win):

    uav_var = np.array([[uav['roll'],uav['pitch'],uav['yaw'],uav['rollRate'],uav['pitchRate'],uav['yawRate'],uav['climbRate'],uav['throttlePct']]])
    scaled_uav = scaler.transform(uav_var)
    pca_uav = pca.transform(scaled_uav)

    flag = tree.predict(pca_uav)[0]
    now = time.time()

    obj =  {
                'time': now,
                'uav_params': uav,
                'uav_scaled': scaled_uav,
                'uav_pca': pca_uav,
                'flag': flag
            }
    uav_stat['data'].append(obj)
    uav_stat[flag] = uav_stat[flag] + 1
    

    if uav_stat['last_state'] != flag:
        uav_stat['last_state'] = flag
        if flag == 'normal':
            print(Fore.BLACK + Back.WHITE + "Normal Pattern")
            print(Style.RESET_ALL)
        elif flag == 'noise':
            print(Fore.BLACK + Back.GREEN + "Noise Pattern")
            print(Style.RESET_ALL)
        elif flag == 'mild':
            print(Fore.BLACK + Back.YELLOW + "Mild Pattern")
            print(Style.RESET_ALL)
        else:
            print(Fore.BLACK + Back.RED + "Found Anomalous Pattern!")
            print(Style.RESET_ALL)

    return uav_stat, True

