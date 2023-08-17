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
    # uav_var = np.array([[uav['roll'],uav['pitch'],uav['yaw'],uav['rollRate'],uav['pitchRate'],uav['yawRate'],0,uav['throttlePct']]])
    # print(uav_var)
    scaled_uav = scaler.transform(uav_var)
    pca_uav = pca.transform(scaled_uav)

    #trocar pela arvore
    # if pca_uav[0][0] <=0:
    #     flag = 'normal'
    # elif pca_uav[0][0] <=1:
    #     flag = 'noise'
    # elif pca_uav[0][0] <=4:
    #     flag = 'mild'
    # else:
    #     flag = 'abnormal'

    flag = tree.predict(pca_uav)[0]
    # if pca_uav[0][0] >= 4:
    #     flag = 'abnormal'
    
    now = time.time()


    # media movel de estatisticas 

    obj =  {
                'time': now,
                'uav_params': uav,
                'uav_scaled': scaled_uav,
                'uav_pca': pca_uav,
                'flag': flag
            }
    uav_stat['data'].append(obj)
    uav_stat[flag] = uav_stat[flag] + 1
    
    # histograma
    # uav_stat_mov[janela] += 1
    # uav_stat_mov[0-10] = %normal, %noise ...
    # uav_stat_mov[11-20] = %normal, %noise ...

    # ok -> nao tem abnormal e ruido (noise+mild)< 40%
    # base -> abnormal < 5% e mild < 40%
    # land -> abnormal > 5% ou mild > 50/70%  

    # [ok ok ok ok ok] -> segue
    # [ok base ok base ok] 

    # a cada 30 segundos 
    # ver se percentuais estao crescendo

    # uav_stat_decision.append()

    # i = 0
    
    # while i < len(uav_stat['data']):
    #     if now - uav_stat['data'][i]['time']  >= time_win:
    #         uav_stat[uav_stat['data'][i]['flag']] = uav_stat[uav_stat['data'][i]['flag']] - 1
    #         uav_stat['data'].pop(i)
    #     else:
    #         i += 1

    # if pca_uav[0][0] <=0:
    #     print(Fore.BLACK + Back.WHITE + "Normal Pattern:")
    #     print(Style.RESET_ALL)
    # elif pca_uav[0][0] <=1:
    #     print(Fore.BLACK + Back.GREEN + "Noise Pattern")
    #     print(Style.RESET_ALL)
    # elif pca_uav[0][0] <=4:
    #     print(Fore.BLACK + Back.YELLOW + "Mild Pattern")
    #     print(Style.RESET_ALL)
    # else:
    #     print(Fore.BLACK + Back.RED + "Found Anomalous Pattern!")
    #     print(Style.RESET_ALL)

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

