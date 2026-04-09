#!/usr/bin/env python3

# ==============================================================================
# MIT License
#
# Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University
# Copyright (c) 2026 Virtual Vehicle Research GmbH
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ==============================================================================

import rclpy
from rclpy.node import Node

import utils
from etsi_its_ivim_ts_msgs.msg import *


class Publisher(Node):

    def __init__(self):

        super().__init__("ivim_ts_publisher")
        self.type = "IVIM_TS"
        topic = "/etsi_its_conversion/ivim_ts/in"
        self.publisher = self.create_publisher(IVIM, topic, 1)
        self.timer = self.create_timer(1.0, self.publish)

    def buildMessage(self):

        msg = IVIM()

        msg.header.protocol_version = 2
        msg.header.message_id = msg.header.MESSAGE_ID_IVIM
        msg.header.station_id.value = 52

        msg.ivi.mandatory.service_provider_id.country_code.value = [0x0D, 0xC0]
        msg.ivi.mandatory.service_provider_id.country_code.bits_unused = 6
        msg.ivi.mandatory.service_provider_id.provider_identifier.value = 25
        msg.ivi.mandatory.ivi_identification_number.value = 1

        msg.ivi.mandatory.time_stamp.value = utils.get_t_its(self.get_clock().now().nanoseconds)
        msg.ivi.mandatory.time_stamp_is_present = True

        msg.ivi.mandatory.valid_from.value = msg.ivi.mandatory.time_stamp.value
        msg.ivi.mandatory.valid_from_is_present = True

        msg.ivi.mandatory.valid_to.value = msg.ivi.mandatory.time_stamp.value + 60000
        msg.ivi.mandatory.valid_to_is_present = True

        msg.ivi.mandatory.ivi_status.value = msg.ivi.mandatory.ivi_status.NEW

        msg.ivi.mandatory.connected_ivi_structures_is_present = True
        connected_ivi = IviIdentificationNumber()
        connected_ivi.value = 2
        msg.ivi.mandatory.connected_ivi_structures.array.append(connected_ivi)

        return msg

    def publish(self):

        msg = self.buildMessage()
        self.get_logger().info(f"Publishing {self.type}")
        self.publisher.publish(msg)


if __name__ == "__main__":

    rclpy.init()
    publisher = Publisher()
    rclpy.spin(publisher)
    rclpy.shutdown()
