#!/usr/bin/env python

class Example:
    def __init__(self, write_lock, bag_file_writer):
        self.name = "example"
        self.bag_file_writer = bag_file_writer

    def trigger_callback(self, testblock_name):
        # Process Trigger
        # e.g. write to bag file
        # self.bag_file_writer.write_to_bagfile(TOPIC, MSG, STAMP)
        pass
