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


def pca_model(filename='pca_model_alt.pkl', path=filesPath):
    """
    Load a Principal Component Analysis (PCA) model from a file.

    Parameters:
    - filename (str): Name of the file containing the PCA model. Default is 'pca_model_alt.pkl'.
    - path (str): Path to the directory where the file is located. Default is `filesPath`.

    Returns:
    - sklearn.decomposition.PCA: The loaded PCA model.

    Note:
    - Ensure that the necessary variables (`filesPath` and `absPath`) are defined before calling this function.
    - The 'joblib' library must be imported in your code for successful execution.

    Example:
    ```python
    # Load PCA model with default filename and path
    my_pca = pca_model()

    # Load PCA model with a custom filename and path
    custom_pca = pca_model(filename='custom_pca_model.pkl', path='/path/to/custom/directory/')
    ```
    """
    # Change the current working directory to the specified path
    os.chdir(path)

    # Load the PCA model from the specified file
    pca = joblib.load(filename)

    # Change the current working directory back to the original path
    os.chdir(absPath)

    # Return the loaded PCA model
    return pca


def scaler_model(filename='scaler_model_alt.pkl', path=filesPath):
    """
    Load a scaler model from a file.

    Parameters:
    - filename (str): Name of the file containing the scaler model. Default is 'scaler_model_alt.pkl'.
    - path (str): Path to the directory where the file is located. Default is `filesPath`.

    Returns:
    - sklearn.preprocessing.StandardScaler: The loaded scaler model.

    Note:
    - Ensure that the necessary variables (`filesPath` and `absPath`) are defined before calling this function.
    - The 'joblib' library must be imported in your code for successful execution.

    Example:
    ```python
    # Load scaler model with default filename and path
    my_scaler = scaler_model()

    # Load scaler model with a custom filename and path
    custom_scaler = scaler_model(filename='custom_scaler_model.pkl', path='/path/to/custom/directory/')
    ```
    """
    # Change the current working directory to the specified path
    os.chdir(path)

    # Load the scaler model from the specified file
    scaler = joblib.load(filename)

    # Change the current working directory back to the original path
    os.chdir(absPath)

    # Return the loaded scaler model
    return scaler


def tree_model(filename='tree_pca_alt.pkl', path=filesPath):
    """
    Load a decision tree model from a file.

    Parameters:
    - filename (str): Name of the file containing the decision tree model. Default is 'tree_pca_alt.pkl'.
    - path (str): Path to the directory where the file is located. Default is `filesPath`.

    Returns:
    - sklearn.tree.DecisionTreeClassifier: The loaded decision tree model.

    Note:
    - Ensure that the necessary variables (`filesPath` and `absPath`) are defined before calling this function.
    - The 'joblib' library must be imported in your code for successful execution.

    Example:
    ```python
    # Load decision tree model with default filename and path
    my_tree = tree_model()

    # Load decision tree model with a custom filename and path
    custom_tree = tree_model(filename='custom_tree_model.pkl', path='/path/to/custom/directory/')
    ```
    """
    # Change the current working directory to the specified path
    os.chdir(path)

    # Load the decision tree model from the specified file
    tree = joblib.load(filename)

    # Change the current working directory back to the original path
    os.chdir(absPath)

    # Return the loaded decision tree model
    return tree



def checkAnomaly(uav, uav_stat, pca, scaler, tree, time_win):
    """
    Check for anomalies in UAV parameters using a trained decision tree model.

    Parameters:
    - uav (dict): Dictionary containing UAV parameters (features).
    - uav_stat (dict): Dictionary storing UAV statistical information and anomaly records.
    - pca (sklearn.decomposition.PCA): PCA model for dimensionality reduction.
    - scaler (sklearn.preprocessing.StandardScaler): Scaler model for feature scaling.
    - tree (sklearn.tree.DecisionTreeClassifier): Trained decision tree model for anomaly detection.
    - time_win (int): Time window for tracking anomalies.

    Returns:
    - uav_stat (dict): Updated UAV statistical information and anomaly records.
    - flag (bool): True if an anomaly is detected, False otherwise.

    Modifies:
    - uav_stat: Updates the 'data' field with the latest anomaly check results.
               Increments the count for the detected anomaly type.

    Note:
    - Ensure that the necessary libraries (`numpy`, `time`, `joblib`) and models are imported in your code.
    - The 'pca' and 'scaler' models should be pre-trained before calling this function.
    """
    uav_var = np.array([[uav['roll'], uav['pitch'], uav['yaw'], uav['heading'], uav['rollRate'], uav['pitchRate'],
                         uav['yawRate'], uav['climbRate'], uav['throttlePct']]])
    scaled_uav = scaler.transform(uav_var)
    pca_uav = pca.transform(scaled_uav)

    flag = tree.predict(pca_uav)[0]
    now = time.time()

    obj = {
        'time': now,
        'uav_params': uav,
        'uav_scaled': scaled_uav,
        'uav_pca': pca_uav,
        'flag': flag
    }
    uav_stat['data'].append(obj)
    uav_stat[flag] = uav_stat[flag] + 1

    return uav_stat, flag


