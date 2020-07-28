import kivy

from kivy.app import App
from kivy.garden.joystick import Joystick
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.lang import Builder
from kivy.properties import BooleanProperty, NumericProperty

# MQTT client
import paho.mqtt.client as mqtt


def on_connect(mqttc, obj, flags, rc):
    # print("rc: " + str(rc))
    pass


def on_message(mqttc, obj, msg):
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))


def on_publish(mqttc, obj, mid):
    # print("mid: " + str(mid))
    pass


def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))


def on_log(mqttc, obj, level, string):
    print(string)


mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe
# Uncomment to enable debug messages
# mqttc.on_log = on_log
mqttc.connect("localhost", 1883, 60)
mqttc.loop_start()

# GUI follows
Builder.load_string('''
#: import Joystick kivy.garden.joystick.Joystick
#: import Knob kivy.garden.knob
<Controller@BoxLayout>:
    joystick: joystick
    label: label
    orientation: 'vertical'

    BoxLayout:
        orientation: 'vertical'

        Joystick:
            id: joystick
            sticky: False
            outer_size: 1
            inner_size: 0.75
            pad_size:   0.5
            outer_line_width: 0.01
            inner_line_width: 0.01
            pad_line_width:   0.01
            outer_background_color: (0.75, 0.75, 0.75, 0.3)
            outer_line_color:       (0.25, 0.25, 0.25, 0.3)
            inner_background_color: (0.75, 0.75, 0.75, 0.1)
            inner_line_color:       (0.7,  0.7,  0.7,  0.1)
            pad_background_color:   (0.4,  0.4,  0.4,  0.3)
            pad_line_color:         (0.35, 0.35, 0.35, 0.3)

        Label:
            id: label
            text: 'Placeholder'

    BoxLayout:
        orientation: 'horizontal'
        ToggleButton:
            text: 'Idle'
            size_hint_y: None
            height: 40
            on_state: root.do_idle('1' if self.state == 'down' else '0')
        ToggleButton:
            text: 'Velocity mode' if self.state == 'down' else 'Time mode'
            size_hint_y: None
            height: 40
            on_state: root.set_mode(True if self.state == 'down' else False)
        BoxLayout:
            orientation: 'vertical'
            Label:
                text: 'Stride: {}%'.format(root.stride_pcnt) if root.velocity_mode else 'Time: {}s'.format(root.stride_time)
                size_hint_x: None
                width: 100
            Knob:
                size: 100, 100
                value: 33
                knobimg_source: "img/knob_black.png"
                markeroff_color: 0.0, 0.0, 0.0, 1
                knobimg_size: 0.9
                on_value: root.change_time(*args)

    BoxLayout:
        orientation: 'horizontal'
        ToggleButton:
            text: 'None'
            group: 'gaits'
            state: 'down'
            size_hint_y: None
            height: 40
            on_state: if self.state == 'down': root.selected_gait(self.text)
        ToggleButton:
            text: 'Wave'
            group: 'gaits'
            size_hint_y: None
            height: 40
            on_state: if self.state == 'down': root.selected_gait(self.text)
        ToggleButton:
            text: 'Tripod'
            group: 'gaits'
            size_hint_y: None
            height: 40
            on_state: if self.state == 'down': root.selected_gait(self.text)
        ToggleButton:
            text: 'TripodRotate'
            group: 'gaits'
            size_hint_y: None
            height: 40
            on_state: if self.state == 'down': root.selected_gait(self.text)
        ToggleButton:
            text: 'WaveRotate'
            group: 'gaits'
            size_hint_y: None
            height: 40
            on_state: if self.state == 'down': root.selected_gait(self.text)
        ToggleButton:
            text: 'Relax'
            group: 'gaits'
            size_hint_y: None
            height: 40
            on_state: if self.state == 'down': root.selected_gait(self.text)
        ToggleButton:
            text: 'Standup'
            group: 'gaits'
            size_hint_y: None
            height: 40
            on_state: if self.state == 'down': root.selected_gait(self.text)


''')


def maprange(a, b, s):
    (a1, a2), (b1, b2) = a, b
    return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))


class Controller(BoxLayout):
    velocity_mode = BooleanProperty(False)
    stride_time = NumericProperty(0.3)
    stride_pcnt = NumericProperty(50)

    gait_lut = {'None': 1, 'Wave': 2, 'Tripod': 3, 'TripodRotate': 4, 'WaveRotate': 5, 'Relax': 7, 'Standup': 8}

    def selected_gait(self, g):
        print("Selected gait: {}".format(g))
        mqttc.publish('quadruped/commands', "G {}".format(self.gait_lut[g]), qos=0)

    def do_idle(self, v):
        mqttc.publish('quadruped/commands', "E {}".format(v), qos=0)

    def change_time(self, arg, v):
        if self.velocity_mode:
            # percentage of maximum stride
            self.stride_pcnt = round(maprange([0, 100], [.1, 100], v), 2)
            mqttc.publish('quadruped/commands', "S {}".format(self.stride_pcnt, qos=0))
        else:
            # time in seconds
            self.stride_time = round(maprange([0, 100], [.01, 1.0], v), 2)
            mqttc.publish('quadruped/commands', "B {}".format(self.stride_time, qos=0))

    def set_mode(self, f):
        self.velocity_mode = f
        print('set velocity mode: {}'.format(self.velocity_mode))
        mqttc.publish('quadruped/commands', "C {}".format(0 if self.velocity_mode else 1), qos=0)


class ControlApp(App):
    def build(self):
        self.root = Controller()
        self.root.joystick.bind(pad=self._update_coordinates)

    def _update_coordinates(self, joystick, pad):
        x = round(pad[0], 3) * 100
        y = round(pad[1], 3) * 100
        magnitude = round(joystick.magnitude, 3)
        text = "x: {}\ny: {}\nmagnitude: {}"
        self.root.label.text = text.format(x, y, magnitude)

        mqttc.publish('quadruped/commands', "X {}".format(x), qos=0)
        mqttc.publish('quadruped/commands', "Y {}".format(y), qos=0)


ControlApp().run()
