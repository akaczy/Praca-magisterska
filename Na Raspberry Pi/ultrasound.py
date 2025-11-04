# file: ultrasonic_bringup/ultrasonic_array.py
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO

SPEED_OF_SOUND = 343.0  # m/s

def now_ns():
    return time.perf_counter_ns()

class MultiGroveUltrasonic(Node):
    def __init__(self):
        super().__init__('ultrasonic_array')

        # ---- parameters (override via YAML) ----
        self.declare_parameter('sig_pins', [26, 19, 27, 4])  # BCM pins
        self.declare_parameter('frames',   ["FL_czujnik", "FR_czujnik", "RR_czujnik", "RL_czujnik"])
        self.declare_parameter('topics',   ['ultrasonic/FL/range','ultrasonic/FR/range','ultrasonic/RR/range','ultrasonic/RL/range'])
        self.declare_parameter('field_of_view', 0.261799)  # ~15 deg
        self.declare_parameter('min_range', 0.03)
        self.declare_parameter('max_range', 3.5)
        self.declare_parameter('echo_timeout_us', 30000)   # 30 ms
        self.declare_parameter('inter_sensor_delay_ms', 60.0)  # spacing between pings
        # ----------------------------------------

        self.sig_pins = [int(x) for x in self.get_parameter('sig_pins').value]
        self.frames   = [str(x) for x in self.get_parameter('frames').value]
        self.topics   = [str(x) for x in self.get_parameter('topics').value]
        self.fov      = float(self.get_parameter('field_of_view').value)
        self.min_r    = float(self.get_parameter('min_range').value)
        self.max_r    = float(self.get_parameter('max_range').value)
        self.echo_to  = int(self.get_parameter('echo_timeout_us').value)
        self.delay_s  = float(self.get_parameter('inter_sensor_delay_ms').value) / 1000.0

        assert len(self.sig_pins)==len(self.frames)==len(self.topics), "sig_pins, frames, topics must match length"

        # GPIO init
        GPIO.setmode(GPIO.BCM)
        for p in self.sig_pins:
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, GPIO.LOW)

        # publishers, one per sensor
        self.pubs = []
        for t in self.topics:
            self.pubs.append(self.create_publisher(Range, t, 10))

        self.idx = 0
        self.timer = self.create_timer(self.delay_s, self._step)

    def destroy_node(self):
        try:
            GPIO.cleanup()
        finally:
            super().destroy_node()

    def measure_pin(self, pin: int):
        """Return distance in meters, NaN on timeout-to-rise, +inf on timeout-to-fall."""
        # 1) 10 µs trigger
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(2e-6)
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(10e-6)
        GPIO.output(pin, GPIO.LOW)

        # 2) listen for echo
        GPIO.setup(pin, GPIO.IN)

        t_deadline = now_ns() + self.echo_to * 1000
        while GPIO.input(pin) == 0:
            if now_ns() > t_deadline:
                return math.nan   # no rising edge

        t_start = now_ns()
        t_deadline = t_start + self.echo_to * 1000
        while GPIO.input(pin) == 1:
            if now_ns() > t_deadline:
                return math.inf   # never fell (no echo within range)

        t_end = now_ns()
        pulse_s = (t_end - t_start) / 1e9
        return (pulse_s * SPEED_OF_SOUND) / 2.0

    def _step(self):
        i = self.idx
        self.idx = (self.idx + 1) % len(self.sig_pins)

        try:
            d = self.measure_pin(self.sig_pins[i])
        except Exception:
            d = math.nan

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frames[i]
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov
        msg.min_range = self.min_r
        msg.max_range = self.max_r

        if math.isnan(d):
            msg.range = float('nan')
        elif math.isinf(d):
            msg.range = float('inf')
        else:
            msg.range = min(max(d, self.min_r), self.max_r)

        self.pubs[i].publish(msg)

def main():
    rclpy.init()
    node = MultiGroveUltrasonic()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
