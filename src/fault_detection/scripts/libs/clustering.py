import os
import joblib

absPath = os.path.dirname(__file__)
relPath = "dependencies"
filesPath = os.path.join(absPath,relPath)

def clusters(filename='clusters.joblib',path=filesPath):
    os.chdir(filesPath)
    clusteringAlg = joblib.load(filename)
    os.chdir(absPath)
    return clusteringAlg