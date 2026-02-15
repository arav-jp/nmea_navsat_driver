# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import time

import serial

import rclpy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from libnmea_navsat_driver.driver import Ros2NMEADriver


def publish_diagnostics(driver, pub, connected, last_data_time, data_timeout):
    """Publish GNSS diagnostic status."""
    diag_msg = DiagnosticArray()
    diag_msg.header.stamp = driver.get_clock().now().to_msg()

    status = DiagnosticStatus()
    status.name = 'GNSS'
    has_error = False

    # RS232通信途絶
    kv = KeyValue()
    kv.key = 'rs232_connection_error'
    kv.value = '1' if not connected else '0'
    status.values.append(kv)
    if not connected:
        has_error = True

    # RS232通信周期
    period_error = False
    if last_data_time is not None:
        period_error = (time.time() - last_data_time) > data_timeout
    else:
        period_error = connected  # connected but never received data
    kv = KeyValue()
    kv.key = 'rs232_period_error'
    kv.value = '1' if period_error else '0'
    status.values.append(kv)
    if period_error:
        has_error = True

    # RS232通信データ & センサエラー (from driver)
    sensor_values = driver.get_diagnostic_values()
    for sensor_kv in sensor_values:
        status.values.append(sensor_kv)
        if sensor_kv.key in ('rs232_data_error', 'custom_error_code') and sensor_kv.value == '1':
            has_error = True

    status.level = DiagnosticStatus.ERROR if has_error else DiagnosticStatus.OK
    status.message = 'ERROR' if has_error else 'OK'

    diag_msg.status.append(status)
    pub.publish(diag_msg)

    driver.reset_diagnostic_flags()


def main(args=None):
    rclpy.init(args=args)

    driver = Ros2NMEADriver()
    frame_id = driver.get_frame_id()

    serial_port = driver.declare_parameter('port', '/dev/ttyUSB0').value
    serial_baud = driver.declare_parameter('baud', 4800).value
    diag_period = driver.declare_parameter('diag_period', 1.0).value
    data_timeout = driver.declare_parameter('data_timeout', 5.0).value

    pub_diagnostics = driver.create_publisher(DiagnosticArray, '/diagnostics', 10)

    serial_connected = False
    last_data_time = None
    last_diag_time = time.time()

    try:
        GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)
        serial_connected = True
        driver.get_logger().info("Successfully connected to {0} at {1}.".format(serial_port, serial_baud))
        try:
            while rclpy.ok():
                data = GPS.readline().strip()
                now = time.time()

                if data:
                    last_data_time = now
                    try:
                        if isinstance(data, bytes):
                            data = data.decode("utf-8")
                        driver.add_sentence(data, frame_id)
                    except ValueError as e:
                        driver.get_logger().warn(
                            "Value error, likely due to missing fields in the NMEA message. Error was: %s. "
                            "Please report this issue at github.com/ros-drivers/nmea_navsat_driver, including a bag file "
                            "with the NMEA sentences that caused it." % e)

                # Periodic diagnostic publishing
                if now - last_diag_time >= diag_period:
                    publish_diagnostics(driver, pub_diagnostics, serial_connected, last_data_time, data_timeout)
                    last_diag_time = now

        except Exception as e:
            driver.get_logger().error("Ros error: {0}".format(e))
            GPS.close()
    except serial.SerialException as ex:
        serial_connected = False
        driver.get_logger().fatal("Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))
        # Publish diagnostic on connection failure
        publish_diagnostics(driver, pub_diagnostics, serial_connected, last_data_time, data_timeout)
