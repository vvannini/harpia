import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import random
import os

from libs import loadData

absPath = os.path.dirname(__file__)
relPath = "dependencies"
filesPath = os.path.join(absPath,relPath)

# def noisyData(data,colName,xi,xf):
#         # print(colName)
#         # print(type(data[colName]))
#         # print(data[colName])
#         noiseModel = np.pi*np.arctan(random.uniform(xi,xf))
        
#         if colName in ['pitch', 'roll', 'yaw', 'heading']:
#             response = noiseModel+float(data[colName])
#         elif colName in ['rollRate', 'pitchRate', 'yawRate']:
#             response = noiseModel/60+float
#         return response

def noisyData(data,colName,xi,xf):

    if colName in ['pitch', 'roll', 'yaw', 'heading']:
        response = float(data[colName]) + random.gauss(-10.0, 10.0)
    elif colName in ['rollRate', 'pitchRate', 'yawRate', 'altitudeRelative']:
        response = float(data[colName]) + random.gauss(-5.0, 5.0)
    elif colName in ['throttlePct', 'climbRate']:
        response = float(data[colName]) + random.gauss(0, 0.5)
    else:
        response = float(data[colName]) + random.gauss(-1.5, 1.5)
    return response


def oneCosineGust(filename='rawData.csv',path=filesPath,print=True):
    os.chdir(filesPath)

    data,_ = loadData.loadData(printColumnNames=True) #loading previous flights statistics
    # data = pd.read_csv(filename,sep=";")[['roll','pitch','heading','rollRate',
    #                                       'pitchRate','yawRate','groundSpeed',
    #                                       'climbRate','altitudeRelative','throttlePct']] # we must NEVER change this! 
    #                                                                                      # (those were the models' dependencies during the previous fitting steps)
    time = [] # time to be appended (in seconds)
    for t in range(0,len(data)):
        time.append(t)
        
    data['time'] = time

    wg0 = max(data['climbRate']) #value of the peak
    Lg = random.uniform(10.,50.) #gust length (m)
    ms2knots = 1.94384 # velocity convertion factor (m/s to knots or TAS)

    data['wg'] = (wg0/2) * (1 - np.cos(2 * np.pi * data['groundSpeed'] * ms2knots * data['time'] / Lg))

    if print == True:
        plt.figure(figsize=(12,8))
        plt.plot(data['time'].head(60),data['climbRate'].head(60),'--')
        plt.plot(data['time'].head(60),data['wg'].head(60))
        plt.xlabel('time [s]')
        plt.ylabel('climbRate [m/s]')
        plt.legend(('Data without noise','Gust effect'),loc='upper left')
        plt.show()


    data['climbRate'] = data['wg']
    data['yaw'] = noisyData(data,'yaw')
    data['roll'] = noisyData(data,'roll')
    data['pitch'] = noisyData(data,'pitch')
    data['rollRate'] = noisyData(data,'rollRate')
    data['pitchRate'] = noisyData(data,'pitchRate')
    data['yawRate'] = noisyData(data,'yawRate')
    data['groundSpeed'] = noisyData(data,'groundSpeed')
    
    return data.drop(['wg','time'],axis=1)