# -*- coding: mbcs -*-
# Beaver_plugin.py - Abaqus/CAE plug-in registration for Beaver

from abaqusGui import getAFXApp,sendCommand
from abaqusConstants import ALL
import os
import sys


def _this_dir():
    try:
        f = globals().get('__file__', '')
        if f:
            return os.path.dirname(os.path.abspath(f))
    except:
        pass
    try:
        return os.getcwd()
    except:
        return '.'


def _prepare_python_path():
    d = _this_dir()
    if d and (d not in sys.path):
        sys.path.insert(0, d)


def _register():
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    cur_dir_str = cur_dir.replace('\\', '/')
    sendCommand("import sys")
    sendCommand("if '%s' not in sys.path: sys.path.insert(0, '%s')" % (cur_dir_str, cur_dir_str))
    _prepare_python_path()
    toolset = getAFXApp().getAFXMainWindow().getPluginToolset()
    toolset.registerKernelMenuButton(
        buttonText='Beaver',
        moduleName='Beaver',
        functionName='launch_gui()',
        applicableModules=ALL,
        version='1.4',
        author='',
        description='Beaver: STL-to-RVE build + EasyPBC + stiffness extraction',
        helpUrl=''
    )


try:
    _register()
except Exception:
    pass
