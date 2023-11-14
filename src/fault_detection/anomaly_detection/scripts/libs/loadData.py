import pandas as pd
from pandas.io.json import json_normalize
import os

absPath = os.path.dirname(__file__)
relPath = "dependencies"
filesPath = os.path.join(absPath,relPath)

def loadData(filename='rawData.csv', path=filesPath, printColumnNames=False):
    """
    Load flight data from a JSON file and return a DataFrame.

    Parameters:
    - filename (str): Name of the JSON file containing the flight data.
    - path (str): Path to the directory containing the JSON file.
    - printColumnNames (bool): If True, print the column names of the loaded data.

    Returns:
    - data (pd.DataFrame): DataFrame containing flight data.
    - statistics (dict): Dictionary with statistics on flight variables (mean and standard deviation).

    Note:
    - The flight data is assumed to be stored in a JSON file with a specific structure.
    - Columns expected in the loaded DataFrame: ['roll', 'pitch', 'yaw', 'heading', 'rollRate', 'pitchRate',
      'yawRate', 'groundSpeed', 'climbRate', 'altitudeRelative', 'throttlePct'].
    - If 'printColumnNames' is True, the column names are printed.

    Example:
    data, statistics = loadData(filename='flight_data.json', path='/path/to/data', printColumnNames=True)
    """
    os.chdir(path)

    # Read JSON file into DataFrame
    df = pd.read_json(filename, lines=True)

    # Explode the "data" column
    df_s = df.explode("data")

    # Reset the index
    df_s = df_s.reset_index(drop=True)

    # Extract nested columns
    df_s["roll"] = df_s["data"].apply(lambda x: x.get("roll"))
    # ... (repeat for other columns)

    # Select the desired columns
    data = df_s[[
        "roll",
        "pitch",
        "yaw",
        "heading",
        "rollRate",
        "pitchRate",
        "yawRate",
        "groundSpeed",
        "climbRate",
        "altitudeRelative",
        "throttlePct"
    ]]

    # Filter out rows with errors
    data = data[~df_s['error']]

    # Calculate statistics
    statistics = {
        'flightVariables': list(data.columns),
        'Avg': list(data.mean()),
        'std': list(data.std())
    }

    if printColumnNames:
        print('\n>> Flight Variables:\n', [col for col in data.columns], '\n')

    return data, statistics

