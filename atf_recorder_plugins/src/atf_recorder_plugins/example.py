#!/usr/bin/env python
from atf_recorder import BagfileWriter


class Example:
    def __init__(self, topic_prefix, config_file, robot_config_file, write_lock, bag_file):
        self.topic_prefix = topic_prefix
        self.test_config = config_file

        self.BfW = BagfileWriter(bag_file, write_lock)

    def trigger_callback(self, msg):
        # Process Trigger
        pass
