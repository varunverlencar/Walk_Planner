#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/plannerplugin')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    plannermodule = RaveCreateModule(env,'plannermodule')
    print plannermodule.SendCommand('help')
finally:
    RaveDestroy()
