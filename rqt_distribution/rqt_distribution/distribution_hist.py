
from python_qt_binding.QtCore import Slot, Qt, qVersion, qWarning, Signal
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import random


colors = ["blue", "orange", "green", "red", "purple",
          "pink", "gray", "olive", "cyan"]
class DistributionHist(QWidget):

    class Canvas(FigureCanvas):
        """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""

        def __init__(self, parent=None):
            super(DistributionHist.Canvas, self).__init__(Figure())
            self.axes = self.figure.add_subplot(111)
            self.axes.grid(True, color='gray')
            self.safe_tight_layout()
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.updateGeometry()

        def resizeEvent(self, event):
            super(DistributionHist.Canvas, self).resizeEvent(event)
            self.safe_tight_layout()

        def safe_tight_layout(self):
            try:
                if self.figure.get_figheight() == 0 or self.figure.get_figwidth() == 0:
                    return
                self.figure.tight_layout()
            except ValueError:
                if parse_version(matplotlib.__version__) >= parse_version('2.2.3'):
                    raise

    limits_changed = Signal()

    def __init__(self, parent=None):
        super(DistributionHist, self).__init__(parent)
        self._canvas = DistributionHist.Canvas()
        self._toolbar = NavigationToolbar(self._canvas, self._canvas)
        vbox = QVBoxLayout()
        vbox.addWidget(self._toolbar)
        vbox.addWidget(self._canvas)
        self.setLayout(vbox)
        self.color = random.choice(colors)

        #self._curves = {}
        #self._current_vline = None
        self._canvas.mpl_connect('button_release_event', self._limits_changed)
        self.data = []
        self.redraw()
        self.show()

    def _limits_changed(self, event):
        self.limits_changed.emit()

    def add_curve(self, curve_id, curve_name, curve_color=QColor(Qt.blue), markers_on=False):
        pass

        # adding an empty curve and change the limits, so save and restore them
        #x_limits = self.get_xlim()
        #y_limits = self.get_ylim()
        #if markers_on:
        #    marker_size = 3
        #else:
        #    marker_size = 0
        #line = self._canvas.axes.plot([], [], 'o-', markersize=marker_size, label=curve_name,
        #                              linewidth=1, picker=5, color=curve_color.name())[0]
        #print("line: ", line)
        #self._canvas.axes.hist([1, 2, 1, 1, 2, 1, 1, 1, 2])
        #mu, sigma = 0, 0.1 # mean and standard deviation
        #s = np.random.normal(mu, sigma, 1000)
        #print(s)
        #self._canvas.axes.hist(s, 30)
        #self._curves[curve_id] = line
        #self._update_legend()
        #self.set_xlim([-5, 5])
        #self.set_ylim([-1, 15])

    def remove_curve(self, curve_id):
        curve_id = str(curve_id)
        if curve_id in self._curves:
            self._curves[curve_id].remove()
            del self._curves[curve_id]
            self._update_legend()

    def _update_legend(self):
        handles, labels = self._canvas.axes.get_legend_handles_labels()
        if handles:
            hl = sorted(zip(handles, labels), key=operator.itemgetter(1))
            handles, labels = zip(*hl)
        self._canvas.axes.legend(handles, labels, loc='upper left')

    def set_values(self, curve, data_x, data_y):
        for y in data_y:
            self.data.append(y)
        if hasattr(self, "hist"):
            self.hist.remove()
        if len(self.data) > 0:
            y_size, x_size, self.hist = self._canvas.axes.hist(self.data,
                                                               bins=30,
                                                               density=True,
                                                               color=self.color)
            y_max = max(y_size)
            y_max_index = [i for i, j in enumerate(y_size) if j == y_max]
            self.set_ylim([-0.1*y_max, 1.1*y_max])
            x_left_border = x_size[y_max_index[0]] - 1.1 *\
                max(x_size[-1] - x_size[y_max_index[0]],
                    x_size[y_max_index[0]] - x_size[0])
            x_right_border = x_size[y_max_index[-1]] + 1.1 *\
                max(x_size[-1] - x_size[y_max_index[-1]],
                    x_size[y_max_index[-1]] - x_size[0])
            self.set_xlim([x_left_border, x_right_border])

    def redraw(self):
        self._canvas.axes.grid(True, color='gray')
        self._canvas.draw()

    #def vline(self, x, color):
    #    # convert color range from (0,255) to (0,1.0)
    #    matcolor = (color[0] / 255.0, color[1] / 255.0, color[2] / 255.0)
    #    if self._current_vline:
    #        self._current_vline.remove()
    #    self._current_vline = self._canvas.axes.axvline(x=x, color=matcolor)

    def set_xlim(self, limits):
        self._canvas.axes.set_xbound(lower=limits[0], upper=limits[1])

    def set_ylim(self, limits):
        self._canvas.axes.set_ybound(lower=limits[0], upper=limits[1])
