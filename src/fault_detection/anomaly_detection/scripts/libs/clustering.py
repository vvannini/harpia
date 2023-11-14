import os
import joblib

absPath = os.path.dirname(__file__)
relPath = "dependencies"
filesPath = os.path.join(absPath,relPath)

def clusters(filename='clusters.joblib', path=filesPath):
    """
    Load a pre-trained clustering algorithm from a joblib file.

    Parameters:
    - filename (str): Name of the joblib file containing the pre-trained clustering algorithm.
    - path (str): Path to the directory containing the joblib file.

    Returns:
    - clusteringAlg: Pre-trained clustering algorithm.

    Note:
    - Ensure that the necessary libraries (`joblib`) are imported in your code.
    - The clustering algorithm should be pre-trained and saved using joblib before calling this function.
    """
    os.chdir(path)
    clusteringAlg = joblib.load(filename)
    os.chdir(absPath)
    return clusteringAlg
