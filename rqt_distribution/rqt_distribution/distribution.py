#!/usr/bin/env python3

from rqt_gui_py.plugin import Plugin
from .distribution_widget import DistributionWidget
from .distribution_hist import DistributionHist

class Distribution(Plugin):
    def __init__(self, context):
        super(Distribution, self).__init__(context)

        self.setObjectName('Distribution')
        self._context = context
        self._node = context.node
        #self._args = self._parse_args(context.argv())
        self._widget = DistributionWidget(self._node)
        self._hist = DistributionHist(self._widget)
        self._widget.switch_data_plot_widget(self._hist)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)
