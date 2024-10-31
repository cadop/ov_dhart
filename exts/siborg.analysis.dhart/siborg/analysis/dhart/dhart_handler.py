''' Handles the global dhart instance to be used within this kit context'''

import sys
from . import core

def get_dhart():
    ''' Get the dhart instance '''
    main_module = sys.modules['__main__']
    instance_name = 'dhart'
    if not hasattr(main_module, 'dhart'):
        setattr(main_module, 'dhart', core.DhartInterface())


    return getattr(main_module, 'dhart')

def remove_dhart():
    ''' Remove the dhart instance '''
    main_module = sys.modules['__main__']
    instance_name = 'dhart'
    if hasattr(main_module, 'dhart'):
        delattr(main_module, 'dhart')