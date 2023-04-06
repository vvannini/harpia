import pandas as pd
import os

absPath = os.path.dirname(__file__)
relPath = "dependencies"
filesPath = os.path.join(absPath,relPath)

def loadData(filename='rawData.csv',path=filesPath,printColumnNames=False):
    os.chdir(filesPath)
    data = pd.read_csv(filename,sep=";")[['roll','pitch','heading','rollRate',
                                          'pitchRate','yawRate','groundSpeed',
                                          'climbRate','altitudeRelative','throttlePct']] # we must NEVER change this! 
                                                                                         # (those were the models' dependencies during the previous fitting steps)
    statistics = {'flightVariables':list(data.columns),
                  'Avg':list(data.mean()),
                  'std':list(data.std())}

    if printColumnNames == True:
        print('\n>> Flight Variables:\n',[col for col in (data.columns)],'\n')

    return data,statistics
