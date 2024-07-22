
import os
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from rqt_py_common.topic_completer import TopicCompleter

from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.utilities import get_message_namespaced_type
from rosidl_runtime_py import import_message_from_namespaced_type

from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget
import re

from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BOOLEAN_TYPE
from rosidl_parser.definition import NamespacedType

from rqt_plot.rosplot import ROSData, RosPlotException


ARRAY_TYPE_REGEX = re.compile(r'(.+)\[(.*)\]')

def _parse_field_name_and_index(field_name):
    # Field names may be indexed, e.g. `my_field[2]`.
    # This parses the actual name and index from the indexed name and returns `field_name, index`.
    # If not indexed, returns `field_name, None`.
    m = ARRAY_TYPE_REGEX.match(field_name)
    if m:
        try:
            return m.group(1), int(m.group(2))
        except ValueError:
            pass
    return field_name, None


def get_plot_fields(node, topic_name):
    topics = node.get_topic_names_and_types()
    real_topic = None
    for name, topic_types in topics:
        candidate = name.split('/')
        desired = topic_name.split('/')
        if candidate == desired[:len(candidate)]:
            real_topic = name
            topic_type_str = topic_types[0] if topic_types else None
            break
    if real_topic is None:
        message = "topic %s does not exist" % (topic_name)
        return [], message

    if topic_type_str is None:
        message = "no topic types found for topic %s " % (topic_name)
        return [], message

    if len(topic_name) < len(real_topic) + 1:
        message = 'no field specified in topic name "{}"'.format(topic_name)
        return [], message

    nested_field_path = topic_name[len(real_topic) + 1:]

    message_class = get_message(topic_type_str)
    if message_class is None:
        message = 'message class "{}" is invalid'.format(topic_type_str)
        return [], message

    nested_fields = iter(f for f in nested_field_path.split('/') if f)
    current_type = get_message_namespaced_type(topic_type_str)
    current_message_class = message_class
    next_field = next(nested_fields, None)
    parsed_fields = []

    while next_field is not None:
        parsed_fields.append(next_field)
        name, index = _parse_field_name_and_index(next_field)
        has_index = index is not None
        base_error_msg = f"trying to parse field '{'.'.join(parsed_fields)}' of topic {real_topic}: "
        no_field_error_msg = base_error_msg + f"'{name}' is not a field of '{topic_type_str}'"

        try:
            slot_index = current_message_class.__slots__.index(f'_{name}')
        except ValueError:
            return [], no_field_error_msg
        current_type = current_message_class.SLOT_TYPES[slot_index]
        is_array_or_sequence = isinstance(current_type, AbstractNestedType)

        if is_array_or_sequence:
            if not has_index:
                return [], base_error_msg + f'{name} is a nested type but not index provided'
            if current_type.has_maximum_size():
                if index >= current_type.maximum_size:
                    return [], (
                        base_error_msg +
                        f"index '{index}' out of bounds, maximum size is {current_type.maximum_size}")
            current_type = current_type.value_type
        elif has_index:
            return [], base_error_msg + "{name} is not an array or sequence"

        if not isinstance(current_type, NamespacedType):
            break
        current_message_class = import_message_from_namespaced_type(current_type)
        next_field = next(nested_fields, None)

    try:
        next_field = next(nested_fields)
        return [], f"'{'.'.join(parsed_fields)}' is a primitive type with no field named '{next_field}'"
    except StopIteration:
        pass

    if isinstance(current_type, AbstractGenericString):
        return [], f"'{topic_name}' is a string, which cannot be plotted"
    if isinstance(current_type, AbstractSequence):
        return [], f"'{topic_name}' is a sequence, which cannot be plotted"
    if isinstance(current_type, Array):
        return (
            [f'{topic_name}[{i}]' for i in range(field_class.maximum_size)],
            f"'{topic_name}' is a fixed size array")
    if isinstance(current_type, NamespacedType):
        plottable_fields = []
        current_message_class = import_message_from_namespaced_type(current_type)
        for n_field, n_current_type in zip(
            current_message_class.__slots__, current_message_class.SLOT_TYPES
        ):
            if isinstance(n_current_type, BasicType):
                plottable_fields.append(n_field[1:])
        if plottable_fields:
            return (
                [f'{topic_name}/{field}' for field in plottable_fields],
                f"{len(plottable_fields)} plottable fields in '{topic_name}'"
            )
    if not isinstance(current_type, BasicType):
        return [], f"{topic_name} cannot be plotted"

    data_kind = 'boolean' if current_type.typename == BOOLEAN_TYPE else 'numeric'
    return [topic_name], f"topic '{topic_name}' is {data_kind}"


def is_plottable(node, topic_name):
    fields, message = get_plot_fields(node, topic_name)
    return len(fields) > 0, message



class DistributionWidget(QWidget):
    _redraw_interval = 400
    def __init__(self, node):
        super(DistributionWidget, self).__init__()

        self._node = node
        self.data_plot = None

        _, package_path = get_resource('packages', 'rqt_distribution')
        ui_file = os.path.join(package_path, 'share', 'rqt_distribution', 'resource', 'distribution.ui')
        loadUi(ui_file, self)
        self._node.get_logger().info("loaded")
        self.setWindowTitle("Distribution")

        self.subscribe_topic_button.setEnabled(False)
        self._topic_completer = TopicCompleter(self.topic_edit)
        self._topic_completer.update_topics(node)
        self.topic_edit.setCompleter(self._topic_completer)
        self._rosdata = {}
        self._remove_topic_menu = QMenu()

        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)


    def switch_data_plot_widget(self, data_plot):
        self.enable_timer(enabled=False)

        self.data_plot_layout.removeWidget(self.data_plot)
        if self.data_plot is not None:
            self.data_plot.close()

        self.data_plot = data_plot
        self.data_plot_layout.addWidget(self.data_plot)
        #self.data_plot.autoscroll(self.autoscroll_checkbox.isChecked())

        # setup drag 'n drop
        #self.data_plot.dropEvent = self.dropEvent
        #self.data_plot.dragEnterEvent = self.dragEnterEvent

        #if self._initial_topics:
        #    for topic_name in self._initial_topics:
        #        self.add_topic(topic_name)
        #    self._initial_topics = None
        #else:
        #    for topic_name, rosdata in self._rosdata.items():
        #        data_x, data_y = rosdata.next()
        #        self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)

        #self._subscribed_topics_changed()
    
    @Slot(str)
    def on_topic_edit_textChanged(self, topic_name):
        # on empty topic name, update topics
        if topic_name in ('', '/'):
            self._topic_completer.update_topics(self._node)

        plottable, message = is_plottable(self._node, topic_name)
        self.subscribe_topic_button.setEnabled(plottable)
        self.subscribe_topic_button.setToolTip(message)

    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.add_topic(str(self.topic_edit.text()))

    def _subscribed_topics_changed(self):
        self._update_remove_topic_menu()
        if not self.pause_button.isChecked():
            # if pause button is not pressed, enable timer based on subscribed topics
            self.enable_timer(self._rosdata)
        self.data_plot.redraw()

    def _update_remove_topic_menu(self):
        def make_remove_topic_function(x):
            return lambda: self.remove_topic(x)

        self._remove_topic_menu.clear()
        for topic_name in sorted(self._rosdata.keys()):
            action = QAction(topic_name, self._remove_topic_menu)
            action.triggered.connect(make_remove_topic_function(topic_name))
            self._remove_topic_menu.addAction(action)

        if len(self._rosdata) > 1:
            all_action = QAction('All', self._remove_topic_menu)
            all_action.triggered.connect(self.clean_up_subscribers)
            self._remove_topic_menu.addAction(all_action)

        self.remove_topic_button.setMenu(self._remove_topic_menu)


    def add_topic(self, topic_name):
        topics_changed = False
        for topic_name in get_plot_fields(self._node, topic_name)[0]:
            if topic_name in self._rosdata:
                qWarning('PlotWidget.add_topic(): topic already subscribed: %s' % topic_name)
                continue
            self._rosdata[topic_name] = ROSData(self._node, topic_name, 0)
            if self._rosdata[topic_name].error is not None:
                qWarning(str(self._rosdata[topic_name].error))
                del self._rosdata[topic_name]
            else:
                data_x, data_y = self._rosdata[topic_name].next()
                self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)
                topics_changed = True

        if topics_changed:
            self._subscribed_topics_changed()

    def update_plot(self):
        if self.data_plot is not None:
            needs_redraw = False
            for topic_name, rosdata in self._rosdata.items():
                try:
                    data_x, data_y = rosdata.next()
                    if data_x or data_y:
                        self.data_plot.set_values(topic_name, data_x, data_y)
                        needs_redraw = True
                except RosPlotException as e:
                    qWarning('PlotWidget.update_plot(): error in rosplot: %s' % e)
            if needs_redraw:
                self.data_plot.redraw()


    def enable_timer(self, enabled=True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()
