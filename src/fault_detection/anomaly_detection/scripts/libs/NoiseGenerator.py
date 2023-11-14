import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import random
import os

absPath = os.path.dirname(__file__)
relPath = "dependencies"
filesPath = os.path.join(absPath,relPath)

# def noisyData(data,colName,xi,xf):
#         # print(colName)
#         # print(type(data[colName]))
#         # print(data[colName])
        
def noisyData(errorType, data,colName,xi,xf):
    """
    Add noise to a given data column based on the specified error type.

    Parameters:
    - errorType (int): Type of error to be added. Possible values: 0, 1, 2, or any other value for a custom noise model.
    - data (pd.Series): The original data column.
    - colName (str): The name of the column to which noise is to be added.
    - xi (float): Lower bound for custom noise model.
    - xf (float): Upper bound for custom noise model.

    Returns:
    - response (float): The data with added noise.

    Note:
    - Supported errorType values:
        - 0: Low noise.
        - 1: Medium noise.
        - 2: High noise.
        - Any other value: Custom noise model based on an arctan function.

    Example:
    response = noisyData(errorType=1, data=df['pitch'], colName='pitch', xi=0.1, xf=0.5)
    """
    if (errorType == 0):
        if colName in ['pitch', 'roll', 'yaw', 'heading']:
            response = float(data[colName]) + random.gauss(0, 3.0)
        elif colName in ['rollRate', 'pitchRate', 'yawRate', 'altitudeRelative']:
            response = float(data[colName]) + random.gauss(0, 2.0)
        elif colName in ['throttlePct', 'climbRate']:
            response = float(data[colName]) + random.gauss(0, 0.2)
        else:
            response = float(data[colName]) + random.gauss(0, 0.8)
        return response

    elif(errorType == 1):
        if colName in ['pitch', 'roll', 'yaw', 'heading']:
            response = float(data[colName]) + random.gauss(.0, 7.0)
        elif colName in ['rollRate', 'pitchRate', 'yawRate', 'altitudeRelative']:
            response = float(data[colName]) + random.gauss(0, 5.0)
        elif colName in ['throttlePct', 'climbRate']:
            response = float(data[colName]) + random.gauss(0, 0.5)
        else:
            response = float(data[colName]) + random.gauss(0, 1.5)
        return response

    elif(errorType == 2):
        if colName in ['pitch', 'roll', 'yaw', 'heading']:
            response = float(data[colName]) + random.gauss(0.0, 15.0)
        elif colName in ['rollRate', 'pitchRate', 'yawRate', 'altitudeRelative']:
            response = float(data[colName]) + random.gauss(0.0, 7.0)
        elif colName in ['throttlePct', 'climbRate']:
            response = float(data[colName]) + random.gauss(0, 1.0)
        else:
            response = float(data[colName]) + random.gauss(0.0, 3.0)
        return response
    else:
        noiseModel = np.pi*np.arctan(random.uniform(xi,xf))
        response = noiseModel*float(data[colName])
        return response

# def oneCosineGust(filename='rawData.csv',path=filesPath,print=True):
#     os.chdir(filesPath)
#     data = pd.read_csv(filename,sep=";")[['roll','pitch','heading','rollRate',
#                                           'pitchRate','yawRate','groundSpeed',
#                                           'climbRate','altitudeRelative','throttlePct']] # we must NEVER change this! 
#                                                                                          # (those were the models' dependencies during the previous fitting steps)
#     time = [] # time to be appended (in seconds)
#     for t in range(0,len(data)):
#         time.append(t)
        
#     data['time'] = time

#     wg0 = max(data['climbRate']) #value of the peak
#     Lg = random.uniform(10.,50.) #gust length (m)
#     ms2knots = 1.94384 # velocity convertion factor (m/s to knots or TAS)

#     data['wg'] = (wg0/2) * (1 - np.cos(2 * np.pi * data['groundSpeed'] * ms2knots * data['time'] / Lg))

#     if print == True:
#         plt.figure(figsize=(12,8))
#         plt.plot(data['time'].head(60),data['climbRate'].head(60),'--')
#         plt.plot(data['time'].head(60),data['wg'].head(60))
#         plt.xlabel('time [s]')
#         plt.ylabel('climbRate [m/s]')
#         plt.legend(('Data without noise','Gust effect'),loc='upper left')
#         plt.show()


#     data['climbRate'] = data['wg']
#     data['roll'] = noisyData(data,'roll')
#     data['pitch'] = noisyData(data,'pitch')
#     data['rollRate'] = noisyData(data,'rollRate')
#     data['pitchRate'] = noisyData(data,'pitchRate')
#     data['yawRate'] = noisyData(data,'yawRate')
#     data['groundSpeed'] = noisyData(data,'groundSpeed')
    
#     return data.drop(['wg','time'],axis=1)