from .extension import *
import os

def package_dir():
    ''' Helper to get directory of this pacckage '''

    return os.path.dirname(os.path.realpath(__file__))