import os
import joblib
import numpy as np
import pandas as pd
import termcolor
from colorama import Fore, Back, Style

absPath = os.path.dirname(__file__)
relPath = "dependencies"
filesPath = os.path.join(absPath,relPath)

def findAnomaly(filename='anomalyDetector.joblib',path=filesPath):
    os.chdir(filesPath)
    classifierAlg = joblib.load(filename)
    os.chdir(absPath)
    return classifierAlg

def clusters(filename='clusters.joblib',path=filesPath):
    os.chdir(filesPath)
    clusteringAlg = joblib.load(filename)
    os.chdir(absPath)
    return clusteringAlg

def checkValues(uav_var, statistics, module1, module2):

    inputArray = np.array([[uav_var['rollRate'],uav_var['pitchRate'],uav_var['yawRate']]])

    anomalyResult = module1.predict(inputArray) #loaded model prediction (anomaly classification)

    if anomalyResult[0] == 0:
        print(Fore.BLACK + Back.GREEN + "Normal Pattern")
        print(Style.RESET_ALL)
# ->->->->
        return None, True
    else:
        print(Fore.BLACK + Back.RED + "Found Anomalous Pattern!")
        print(Style.RESET_ALL)
        print('\n>> Starting clustering...')

        # inputList = [] # will append the current flight data (all of them)
        # for c in range(clusteringInput.shape[1]): #columns
        #     inputList.append(clusteringInput[r][c])
        # print(f'\n* roll:{round(inputList[0],3)}dg\n* pitch:{round(inputList[1],3)}dg\n* heading:{round(inputList[2],3)}dg\n* rollRate:{round(inputList[3],3)}dg/s\n* pitchRate:{round(inputList[4],3)}dg/s\n* yawRate:{round(inputList[5],3)}dg/s\n* groundSpeed:{round(inputList[6],3)}m/s\n* climbRate:{round(inputList[7],3)}m/s\n* altitudeRelative:{round(inputList[8],3)}m\n* throttlePct:{round(inputList[9],3)}%')
        
        inputArray = np.array([[uav_var['roll'],uav_var['pitch'],uav_var['heading'],
                                uav_var['rollRate'],uav_var['pitchRate'],uav_var['yawRate'],uav_var['groundSpeed'],
                                uav_var['climbRate'],uav_var['altitudeRelative'],uav_var['throttlePct']]])

        clusteringResult = module2.predict(inputArray) #loaded model prediction (anomaly clustering)
        print(f'\n>> Main Cluster (k-Means):{clusteringResult}')

        # For each 3 sequential candidates, we need to get a Weighted Cluster to take our real-time decision
        sequential = {'roll':[],
                      'pitch':[],
                      'heading':[],
                      'rollRate':[],
                      'pitchRate':[],
                      'yawRate':[],
                      'groundSpeed':[],
                      'climbRate':[],
                      'altitudeRelative':[],
                      'throttlePct':[]};weightedCluster = []
        
        #------------#
        ## 0.roll:
        rollAvg = statistics['Avg'][0];rollStd = statistics['std'][0] #mean() and std()
        upperBoundary_roll = rollAvg + 3*rollStd;lowerBoundary_roll = rollAvg - 3*rollStd #boundaries: mean() +/- 3std()
        
        if lowerBoundary_roll <= uav_var['roll'] <= upperBoundary_roll:
            sequential['roll'].append('ok') #normal range (level=0)
        elif (lowerBoundary_roll-rollStd) <= uav_var['roll'] < lowerBoundary_roll or upperBoundary_roll < uav_var['roll'] <= (upperBoundary_roll + rollStd):
            sequential['roll'].append('mild') #over normal boundaries limited by 1std (level=1)
        elif (lowerBoundary_roll-2*rollStd) <= uav_var['roll'] < (lowerBoundary_roll-rollStd) or (upperBoundary_roll + rollStd) < uav_var['roll'] <= (upperBoundary_roll+2*rollStd):
            sequential['roll'].append('moderate') #between 1std and 2std (level=2)
        else:
            sequential['roll'].append('critical') #greater than 2std (level=3)
        
        #------------#
        ## 1.pitch:
        pitchAvg = statistics['Avg'][1];pitchStd = statistics['std'][1]
        upperBoundary_pitch = pitchAvg + 3*pitchStd;lowerBoundary_pitch = pitchAvg - 3*pitchStd
        
        if lowerBoundary_pitch <= uav_var['pitch'] <= upperBoundary_pitch:
            sequential['pitch'].append('ok')
        elif (lowerBoundary_pitch-pitchStd) <= uav_var['pitch'] < lowerBoundary_pitch or upperBoundary_pitch < uav_var['pitch'] <= (upperBoundary_pitch + pitchStd):
            sequential['pitch'].append('mild')
        elif (lowerBoundary_pitch-2*pitchStd) <= uav_var['pitch'] < (lowerBoundary_pitch-pitchStd) or (upperBoundary_pitch + pitchStd) < uav_var['pitch'] <= (upperBoundary_pitch+2*pitchStd):
            sequential['pitch'].append('moderate')
        else:
            sequential['pitch'].append('critical')

        #------------#
        ## 2.heading:
        headingAvg = statistics['Avg'][2];headingStd = statistics['std'][2]
        upperBoundary_heading = headingAvg + 3*headingStd;lowerBoundary_heading = headingAvg - 3*headingStd

        if lowerBoundary_heading <= uav_var['heading'] <= upperBoundary_heading:
            sequential['heading'].append('ok')
        elif (lowerBoundary_heading-headingStd) <= uav_var['heading'] < lowerBoundary_heading or upperBoundary_heading < uav_var['heading'] <= (upperBoundary_heading + headingStd):
            sequential['heading'].append('mild')
        elif (lowerBoundary_heading-2*headingStd) <= uav_var['heading'] < (lowerBoundary_heading-headingStd) or (upperBoundary_pitch + headingStd) < uav_var['heading'] <= (upperBoundary_heading+2*headingStd):
            sequential['heading'].append('moderate')
        else:
            sequential['heading'].append('critical')

        #------------#
        ## 3.rollRate:
        rollRateAvg = statistics['Avg'][3];rollRateStd = statistics['std'][3]
        upperBoundary_rollRate = rollRateAvg + 3*rollRateStd;lowerBoundary_rollRate = rollRateAvg - 3*rollRateStd

        if lowerBoundary_rollRate <= uav_var['rollRate'] <= upperBoundary_rollRate:
            sequential['rollRate'].append('ok')
        elif (lowerBoundary_rollRate-rollRateStd) <= uav_var['rollRate'] < lowerBoundary_rollRate or upperBoundary_rollRate < uav_var['rollRate'] <= (upperBoundary_rollRate + rollRateStd):
            sequential['rollRate'].append('mild')
        elif (lowerBoundary_rollRate-2*rollRateStd) <= uav_var['rollRate'] < (lowerBoundary_rollRate-rollRateStd) or (upperBoundary_rollRate + rollRateStd) < uav_var['rollRate'] <= (upperBoundary_rollRate+2*rollRateStd):
            sequential['rollRate'].append('moderate')
        else:
            sequential['rollRate'].append('critical')

        #------------#
        ## 4.pitchRate
        pitchRateAvg = statistics['Avg'][4];pitchRateStd = statistics['std'][4]
        upperBoundary_pitchRate = pitchRateAvg + 3*pitchRateStd;lowerBoundary_pitchRate = pitchRateAvg - 3*pitchRateStd

        if lowerBoundary_pitchRate <= uav_var['pitchRate'] <= upperBoundary_pitchRate:
            sequential['pitchRate'].append('ok')
        elif (lowerBoundary_pitchRate-pitchRateStd) <= uav_var['pitchRate'] < lowerBoundary_pitchRate or upperBoundary_pitchRate < uav_var['pitchRate'] <= (upperBoundary_pitchRate + pitchRateStd):
            sequential['pitchRate'].append('mild')
        elif (lowerBoundary_pitchRate-2*pitchRateStd) <= uav_var['pitchRate'] < (lowerBoundary_pitchRate-pitchRateStd) or (upperBoundary_pitchRate + pitchRateStd) < uav_var['pitchRate'] <= (upperBoundary_pitchRate+2*pitchRateStd):
            sequential['pitchRate'].append('moderate')
        else:
            sequential['pitchRate'].append('critical')

        #------------#
        ## 5.yawRate:
        yawRateAvg = statistics['Avg'][5];yawRateStd = statistics['std'][5]
        upperBoundary_yawRate = yawRateAvg + 3*yawRateStd;lowerBoundary_yawRate = yawRateAvg - 3*yawRateStd

        if lowerBoundary_yawRate <= uav_var['yawRate'] <= upperBoundary_yawRate:
            sequential['yawRate'].append('ok')
        elif (lowerBoundary_yawRate-yawRateAvg) <= uav_var['yawRate'] < lowerBoundary_yawRate or upperBoundary_yawRate < uav_var['yawRate'] <= (upperBoundary_yawRate + yawRateAvg):
            sequential['yawRate'].append('mild')
        elif (lowerBoundary_yawRate-2*yawRateStd) <= uav_var['yawRate'] < (lowerBoundary_yawRate-yawRateStd) or (upperBoundary_yawRate + yawRateStd) < uav_var['yawRate'] <= (upperBoundary_yawRate+2*yawRateStd):
            sequential['yawRate'].append('moderate')
        else:
            sequential['yawRate'].append('critical')

        
        #------------#
        ## 6.groundSpeed:
        groundSpeedAvg = statistics['Avg'][6];groundSpeedStd = statistics['std'][6]
        upperBoundary_groundSpeed = groundSpeedAvg + 3*groundSpeedStd;lowerBoundary_groundSpeed = groundSpeedAvg - 3*groundSpeedStd

        if lowerBoundary_groundSpeed <= uav_var['groundSpeed'] <= upperBoundary_groundSpeed:
            sequential['groundSpeed'].append('ok')
        elif (lowerBoundary_groundSpeed-groundSpeedAvg) <= uav_var['groundSpeed'] < lowerBoundary_groundSpeed or upperBoundary_groundSpeed < uav_var['groundSpeed'] <= (upperBoundary_groundSpeed + groundSpeedAvg):
            sequential['groundSpeed'].append('mild')
        elif (lowerBoundary_groundSpeed-2*groundSpeedStd) <= uav_var['groundSpeed'] < (lowerBoundary_groundSpeed-groundSpeedStd) or (upperBoundary_groundSpeed + groundSpeedStd) < uav_var['groundSpeed'] <= (upperBoundary_groundSpeed+2*groundSpeedStd):
            sequential['groundSpeed'].append('moderate')
        else:
            sequential['groundSpeed'].append('critical')

        #------------#
        ## 7.climbRate
        climbRateAvg = statistics['Avg'][7];climbRateStd = statistics['std'][7]
        upperBoundary_climbRate = climbRateAvg + 3*climbRateStd;lowerBoundary_climbRate = climbRateAvg - 3*climbRateStd

        if lowerBoundary_climbRate <= uav_var['climbRate'] <= upperBoundary_climbRate:
            sequential['climbRate'].append('ok')
        elif (lowerBoundary_climbRate-climbRateAvg) <= uav_var['climbRate'] < lowerBoundary_climbRate or upperBoundary_climbRate < uav_var['climbRate'] <= (upperBoundary_climbRate + climbRateAvg):
            sequential['climbRate'].append('mild')
        elif (lowerBoundary_climbRate-2*climbRateStd) <= uav_var['climbRate'] < (lowerBoundary_climbRate-climbRateStd) or (upperBoundary_climbRate + climbRateStd) < uav_var['climbRate'] <= (upperBoundary_climbRate+2*climbRateStd):
            sequential['climbRate'].append('moderate')
        else:
            sequential['climbRate'].append('critical')


        #------------#
        ## 8.altitudeRelative:
        altitudeRelativeAvg = statistics['Avg'][8];altitudeRelativeStd = statistics['std'][8]
        upperBoundary_altitudeRelative = altitudeRelativeAvg + 3*altitudeRelativeStd;lowerBoundary_altitudeRelative = altitudeRelativeAvg - 3*altitudeRelativeStd

        if lowerBoundary_altitudeRelative <= uav_var['altitudeRelative'] <= upperBoundary_altitudeRelative:
            sequential['altitudeRelative'].append('ok')
        elif (lowerBoundary_altitudeRelative-altitudeRelativeAvg) <= uav_var['altitudeRelative'] < lowerBoundary_altitudeRelative or upperBoundary_altitudeRelative < uav_var['altitudeRelative'] <= (upperBoundary_altitudeRelative + altitudeRelativeAvg):
            sequential['altitudeRelative'].append('mild')
        elif (lowerBoundary_altitudeRelative-2*altitudeRelativeStd) <= uav_var['altitudeRelative'] < (lowerBoundary_altitudeRelative-altitudeRelativeStd) or (upperBoundary_altitudeRelative + altitudeRelativeStd) < uav_var['altitudeRelative'] <= (upperBoundary_altitudeRelative+2*altitudeRelativeStd):
            sequential['altitudeRelative'].append('moderate')
        else:
            sequential['altitudeRelative'].append('critical')

        #------------#
        ## 9.throttlePct:
        throttlePctAvg = statistics['Avg'][9];throttlePctStd = statistics['std'][9]
        upperBoundary_throttlePct = throttlePctAvg + 3*throttlePctStd;lowerBoundary_throttlePct = throttlePctAvg - 3*throttlePctStd

        if lowerBoundary_throttlePct <= uav_var['throttlePct'] <= upperBoundary_throttlePct:
            sequential['throttlePct'].append('ok')
        elif (lowerBoundary_throttlePct-throttlePctAvg) <= uav_var['throttlePct'] < lowerBoundary_throttlePct or upperBoundary_throttlePct < uav_var['throttlePct'] <= (upperBoundary_throttlePct + throttlePctAvg):
            sequential['throttlePct'].append('mild')
        elif (lowerBoundary_throttlePct-2*throttlePctStd) <= uav_var['throttlePct'] < (lowerBoundary_throttlePct-throttlePctStd) or (upperBoundary_throttlePct + throttlePctStd) < uav_var['throttlePct'] <= (upperBoundary_throttlePct+2*throttlePctStd):
            sequential['throttlePct'].append('moderate')
        else:
            sequential['throttlePct'].append('critical')

        #------------#
        ## weighted cluster:
        weightedCluster.append(pd.DataFrame(sequential).mode(axis=0).values[0][0]) #weighted cluster over previous data
        

        print(f'\n>> Individuals:\n{pd.DataFrame(sequential)}')
        if weightedCluster[0] == 'ok':
            print('\nWeighted Cluster: ' + termcolor.colored("Normal Level", "green", attrs=['bold']) + '\n') #keep flying
        elif weightedCluster[0] == 'mild':
            print('\nWeighted Cluster: ' + termcolor.colored("Mild Level", "blue", attrs=['bold']) + '\n') #keep flying cautiously
        elif weightedCluster[0] == 'moderate':
            print('\nWeighted Cluster: ' + termcolor.colored("Moderate Level", "yellow", attrs=['bold'])) #go to the next waypoint
        else:
            print('\nWeighted Cluster: ' + termcolor.colored("Critical Level", "red", attrs=['bold']) + '\n') #land on this location
            abort = input('\n>> Abort Mission? [Y/N] -> ').upper() #user decision
            return abort, False
        return weightedCluster, False