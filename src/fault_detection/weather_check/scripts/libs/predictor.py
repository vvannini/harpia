
import os
import pickle

absPath = os.path.dirname(__file__)
relPath = "dependencies"
filesPath = os.path.join(absPath,relPath)


def GradientBooster(filename='GradientBooster.pkl',path=filesPath):
    os.chdir(filesPath)
    reg_model = pickle.load(open(filename, 'rb'))
    os.chdir(absPath)
    return reg_model