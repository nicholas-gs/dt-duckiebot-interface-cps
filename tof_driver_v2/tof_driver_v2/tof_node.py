#!/usr/bin/env python3

import smbus
import rclpy
import yaml
import os

from rclpy.node import Node
from collections import namedtuple

# from dt_interfaces_cps.msg import ToFStamped
from tof_driver_v2.tof import ToF
from sensor_msgs.msg import Range
from std_msgs.msg import Header


MUX_ADDR = 0x70
TOF_CHIP_ID = 0xAD02


class SensorData(namedtuple('sensor_data', ['index', 'tof', 'pub'])):
    """ Represents all of the necessary data for communicating with 1 individual
    ToF sensor.
    index: 0 - 7 inclusive. Index on the I2C multiplexer
    tof: The actual ToF object
    pub: rospy publisher
    """


class ToFNode(Node):
    """This node performs I2C communication with RFD77402 time-of-flight (ToF)
    distance sensors. This node will try to communicate with each of the 8 ports
    on the multiplexer on the front bumper.
    For each ToF sensor it finds, it will publish a ROS topic called 'tof_<n>',
    where <n> is the number of the port on the multiplexer.
    If it finds no ToF sensors, this node should cleanly exit. If an uncaught
    exception results in a stack trace being displayed, this is a bug, and
    should be reported.
    """ 

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._retrieve_parameters()

        self.smbus = smbus.SMBUS(1)

        self.tofs = []

        for i in range(0, 8):
            try:
                self.select_tof(i)
                tof = ToF()
                chip_id = self.smbus.read_word_data(
                    tof.addr, tof.RFD77402_MOD_CHIP_ID)
                if chip_id == TOF_CHIP_ID:
                    self.get_logger().info(f"Found a ToF chip at index {i}")
                    pub = self.create_publisher(Range, f"tof_{i}", 10)
                    self.tofs.append(SensorData(i, tof, pub))
                    tof.begin()
            except IOError:
                pass
        
        if len(self.tofs) == 0:
            raise RuntimeError("No valid ToF sensors found")

        self.timer = self.create_timer(1.0/self._polling_hz,
            self.read_distances)

    def _retrieve_parameters(self):
        # get parameters
        self.declare_parameter("veh")
        self.declare_parameter("file_path")
        self.declare_parameter("polling_hz")
        self.declare_parameter("m")
        self.declare_parameter("b")

        self._veh_name = self.get_parameter("veh")\
            .get_parameter_value().string_value
        self._file_path = "/data/config/calibrations/sensor_suite/tof/{}.yaml"\
            .format(self._veh_name)
        self._polling_hz = self.get_parameter("polling_hz")\
            .get_parameter_value().integer_value
        self._m = self.get_parameter("m")\
            .get_parameter_value().double_array_value
        self._b = self.get_parameter("b")\
            .get_parameter_value().double_array_value

    def select_tof(self, index):
        """Selects which port of the multiplexer is currently being used.
        :param index: Index of the port on the multiplexer (0 - 7, inclusive)
        :return: None
        :raises: IOError if it is unable to communicate with the front
            bumper multiplexer
        """
        self.smbus.write_byte(MUX_ADDR, 1 << index)

    def read_distances(self):
        """Reads all connected ToF sensors and publishes the results.
        This function should be called periodically by a ROSPy Timer.
        """
        for sensor in self.tofs:
            try:
                self.select_tof(sensor.index)
                error_code, distance, valid_pixels, confidence_value = sensor.tof.takeMeasurement()
                distance = self._m[sensor.index]*distance + self._b[sensor.index]
            except IOError as e:
                self.get_logger().error("IOError when reading from ToF: {e}")
                error_code, distance, valid_pixels, confidence_value = 0x03, 0, 0, 0
            finally:
                msg = Range(header=Header(stamp=self.get_clock().now().to_msg(),
                    frame_id=f"{self._veh}/tof/{self._sensor_name}"),
                    radiation_type=Range.INFRARED,
                    field_of_view=self._accuracy.fov,
                    min_range=self._accuracy.min_range,
                    max_range=self._accuracy.max_range,
                    range=distance)
                sensor.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    tof_node = ToFNode(node_name="tof_node")
    try:
        rclpy.spin(tof_node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        pass
    finally:
        tof_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
