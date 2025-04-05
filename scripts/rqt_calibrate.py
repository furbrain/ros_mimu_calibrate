#!/usr/bin/env python

import sys

from mimu_calibrate.cal_plugin import MyPlugin
from rqt_gui.main import Main

plugin = 'rqt_calibrate.py'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))