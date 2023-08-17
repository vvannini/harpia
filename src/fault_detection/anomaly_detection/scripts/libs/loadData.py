import pandas as pd
from pandas.io.json import json_normalize
import os

absPath = os.path.dirname(__file__)
relPath = "dependencies"
filesPath = os.path.join(absPath,relPath)

def loadData(filename='rawData.csv',path=filesPath,printColumnNames=False):
    os.chdir(filesPath)
    # data = pd.read_csv(filename,sep=";")[['roll','pitch','heading','rollRate',
    #                                       'pitchRate','yawRate','groundSpeed',
    #                                       'climbRate','altitudeRelative','throttlePct']] # we must NEVER change this! 
    #                                                                                      # (those were the models' dependencies during the previous fitting steps)

    # Read JSON file into DataFrame
    json_file_path = "/home/vannini/harpia/flight_data.json"
    df = pd.read_json(json_file_path, lines=True)

    # Explode the "data" column
    df_s = df.explode("data")

    # Reset the index
    df_s = df_s.reset_index(drop=True)

    # Extract nested columns
    df_s["roll"] = df_s["data"].apply(lambda x: x.get("roll"))
    df_s["pitch"] = df_s["data"].apply(lambda x: x.get("pitch"))
    df_s["yaw"] = df_s["data"].apply(lambda x: x.get("yaw"))
    df_s["heading"] = df_s["data"].apply(lambda x: x.get("heading"))
    df_s["rollRate"] = df_s["data"].apply(lambda x: x.get("rollRate"))
    df_s["pitchRate"] = df_s["data"].apply(lambda x: x.get("pitchRate"))
    df_s["yawRate"] = df_s["data"].apply(lambda x: x.get("yawRate"))
    df_s["groundSpeed"] = df_s["data"].apply(lambda x: x.get("groundSpeed"))
    df_s["climbRate"] = df_s["data"].apply(lambda x: x.get("climbRate"))
    df_s["altitudeRelative"] = df_s["data"].apply(lambda x: x.get("altitudeRelative"))
    df_s["throttlePct"] = df_s["data"].apply(lambda x: x.get("throttlePct"))
    df_s = df_s[~df_s['error']]
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

    statistics = {'flightVariables':list(data.columns),
                  'Avg':list(data.mean()),
                  'std':list(data.std())}

    if printColumnNames == True:
        print('\n>> Flight Variables:\n',[col for col in (data.columns)],'\n')

    return data,statistics
