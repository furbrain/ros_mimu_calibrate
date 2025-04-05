import os
from functools import partial

import rospy
import rospkg
import rosparam

from std_srvs.srv import Trigger
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

class MyPlugin(Plugin):
    SERVICES = ["start_recording",
                "get_variance",
                "calibrate",]

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('mimu_calibrate'), 'resources', 'rqt_plugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Calibration')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        for srv in self.SERVICES:
            widget = getattr(self._widget,srv+'_but')
            widget.clicked.connect(partial(self._but_clicked,srv))
        self._widget.save_calibration_but.clicked.connect(self.save_calibration)

    def _but_clicked(self, service: str):
        try:
            result = rospy.ServiceProxy(service, Trigger).call()
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        else:
            print("Success!: ", result)

    def save_calibration(self):
        print("Saving Calibration")
        ns = rospy.get_namespace()[1:] # skip first / so os path join doesn't get confused...
        if "ROS_HOME" in os.environ:
            fname = os.path.join(os.environ["ROS_HOME"], ns, "calibrate.yaml")
        else:
            fname = os.path.join(os.environ["HOME"], ".ros", ns, "calibration.yaml")
        rosparam.dump_params(fname, "calibration")

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
