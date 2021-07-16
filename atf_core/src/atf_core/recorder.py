#!/usr/bin/env python
import os
import roslib
import rospy
import rospkg
import rostopic
import rosbag
import tf
import tf2_ros
import yaml

from threading import Lock
from rospy.exceptions import ROSException

from tf2_msgs.msg import TFMessage
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray

from actionlib.simple_action_client import SimpleActionClient
from atf_core.bagfile_helper import BagfileWriter
from atf_core.error import ATFRecorderError
import atf_recorder_plugins

class ATFRecorder:
    def __init__(self, test):
        self.test = test
        #print "recorder_core: self.test.name:", self.test.name

        recorder_config = self.load_data(rospkg.RosPack().get_path("atf_recorder_plugins") +
                                         "/config/recorder_plugins.yaml")

        try:
            # delete test_results directories and create new ones
            if os.path.exists(self.test.generation_config["bagfile_output"]):
            #    shutil.rmtree(self.tests.generation_config"bagfile_output"]) #FIXME will fail if multiple test run concurrently
                pass
            else:
                os.makedirs(self.test.generation_config["bagfile_output"])
        except OSError:
            pass

        # lock for self variables of this class
        self.lock = Lock()

        # create bag file writer handle
        self.lock_write = Lock()
        self.bag = rosbag.Bag(self.test.generation_config["bagfile_output"] + self.test.name + ".bag", 'w')
        self.bag_file_writer = BagfileWriter(self.bag, self.lock_write)

        # Init metric recorder
        self.recorder_plugin_list = []
        #print "recorder_config", recorder_config
        if len(recorder_config) > 0:
            for value in list(recorder_config.values()):
                #print "value=", value
                self.recorder_plugin_list.append(getattr(atf_recorder_plugins, value)(self.lock_write,
                                                                                      self.bag_file_writer))
        #print "self.recorder_plugin_list", self.recorder_plugin_list

        #rospy.Service(self.topic + "recorder_command", RecorderCommand, self.command_callback)
        self.diagnostics = None
        self.diagnostics_agg = None
        rospy.Subscriber("/diagnostics_toplevel_state", DiagnosticStatus, self.diagnostics_callback)
        rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.diagnostics_agg_callback)
        rospy.on_shutdown(self.shutdown)
        
        # wait for topics, services, actions and tfs to become active
        wait_timeout = None # None is infinite timeout
        start_time = rospy.Time.now()
        if test.robot_config != None and 'wait_timeout' in test.robot_config:
            if test.robot_config["wait_timeout"] > 0:
                wait_timeout = test.robot_config["wait_timeout"]
                rospy.loginfo("wait_timeout is set to %s", str(wait_timeout))

        if test.robot_config != None and 'wait_for_topics' in test.robot_config:
            for topic in test.robot_config["wait_for_topics"]:
                try:
                    rospy.loginfo("Waiting for topic '%s'...", topic)
                    self.check_for_timeout(start_time, wait_timeout)
                    rospy.wait_for_message(topic, rospy.AnyMsg, wait_timeout)
                    rospy.loginfo("... got message on topic '%s'.", topic)
                except ROSException as e:
                    msg = "... wait_timeout of {} sec exceeded during wait_for_topics: {}".format(wait_timeout, e)
                    rospy.logerr(msg)
                    raise ATFRecorderError(msg)

        if test.robot_config != None and 'wait_for_services' in test.robot_config:
            for service in test.robot_config["wait_for_services"]:
                try:
                    rospy.loginfo("Waiting for service '%s'...", service)
                    self.check_for_timeout(start_time, wait_timeout)
                    rospy.wait_for_service(service, wait_timeout)
                    rospy.loginfo("... service '%s' available.", service)
                except ROSException as e:
                    msg = "... wait_timeout of {} sec exceeded during wait_for_services: {}".format(wait_timeout, e)
                    rospy.logerr(msg)
                    raise ATFRecorderError(msg)

        if test.robot_config != None and 'wait_for_actions' in test.robot_config:
            for action in test.robot_config["wait_for_actions"]:
                rospy.loginfo("Waiting for action '%s'...", action)
                self.check_for_timeout(start_time, wait_timeout)
                try:
                    # wait for action status topic
                    rospy.wait_for_message(action + "/status", rospy.AnyMsg, wait_timeout)
                except ROSException as e:
                    msg = "... wait_timeout of {} sec exceeded during wait_for_actions: {}".format(wait_timeout, e)
                    rospy.logerr(msg)
                    raise ATFRecorderError(msg)

                # get action type of goal topic
                topic_type = rostopic._get_topic_type(action + "/goal")[0]

                # remove "Goal" string from action type
                if topic_type == None or not "Goal" in topic_type:  ## pylint: disable=unsupported-membership-test
                    msg = "Could not get type for action %s. type is %s"%(action, topic_type)
                    rospy.logerr(msg)
                    raise ATFRecorderError(msg)
                # remove "Goal" from type
                topic_type = topic_type[0:len(topic_type)-4] ## pylint: disable=unsubscriptable-object
                client = SimpleActionClient(action, roslib.message.get_message_class(topic_type))

                # wait for action server
                if wait_timeout == None:
                    success = client.wait_for_server()
                else:
                    success = client.wait_for_server(rospy.Duration(wait_timeout))
                if success:
                    rospy.loginfo("... action '%s' available.", action)
                else:
                    msg = "... wait_timeout of {} sec exceeded during wait_for_actions: server not available".format(wait_timeout)
                    rospy.logerr(msg)
                    raise ATFRecorderError(msg)

        if test.robot_config != None and 'wait_for_tfs' in test.robot_config:
            listener = tf.TransformListener()
            for root_frame, measured_frame in test.robot_config["wait_for_tfs"]:
                rospy.loginfo("Waiting for transformation from '%s' to '%s' ...", root_frame, measured_frame)
                while not rospy.is_shutdown():
                    msg = "... wait_timeout of {} sec exceeded during wait_for_tfs".format(wait_timeout)
                    self.check_for_timeout(start_time, wait_timeout, message=msg)
                    try:
                        listener.waitForTransform(root_frame, measured_frame, rospy.Time(), rospy.Duration(1))
                        break # transform is available
                    except tf2_ros.TransformException as e: #pylint: disable=no-member
                        pass
                    rospy.logdebug("... waiting since %.1f sec for transformation from '%s' to '%s' to become available ..."%((rospy.Time.now() - start_time).to_sec(), root_frame, measured_frame))
                rospy.loginfo("... transformation from '%s' to '%s' available.", root_frame, measured_frame)

        if test.robot_config != None and 'wait_for_diagnostics' in test.robot_config and test.robot_config["wait_for_diagnostics"]:
            rospy.loginfo("Waiting for diagnostics to become OK ...")
            r = rospy.Rate(100)
            while not rospy.is_shutdown() and self.diagnostics != None and self.diagnostics.level != DiagnosticStatus.OK:
                msg = "... wait_timeout of {} sec exceeded during wait_for_diagnostics. Latest diagnostic failures are:\n{}".format(wait_timeout, self.filter_diagnostics_agg(self.diagnostics_agg))
                self.check_for_timeout(start_time, wait_timeout, message=msg)
                rospy.logdebug("... waiting since %.1f sec for diagnostics to become OK ...", (rospy.Time.now() - start_time).to_sec())
                r.sleep()
            rospy.loginfo("... diagnostics are OK.")

        self.active_topics = {}

        # special case for tf: make sure tf_buffer is already filled (even before the testblocks are started). Thus we need to record /tf and /tf_static all the time, even if there is no active testblock using tf.
        for testblock in self.test.testblocks:
            topics = self.get_topics_of_testblock(testblock.name)
            if "/tf" in topics or "/tf_static" in topics:
                self.active_topics["/tf"]           = ["always"]
                self.active_topics["/tf_static"]    = ["always"]

        self.subscribers = {}
        self.tf_static_message = TFMessage()

        rospy.Timer(rospy.Duration(0.1), self.create_subscriber_callback)
        rospy.Timer(rospy.Duration(0.05), self.tf_static_timer_callback)

        rospy.loginfo("ATF recorder: started!")

    def check_for_timeout(self, start_time, timeout, message=None):
        if timeout != None and rospy.Time.now() - start_time > rospy.Duration(timeout):
            msg = message if message else "... wait_timeout of %.1f sec exceeded."%timeout
            rospy.logerr(msg)
            raise ATFRecorderError(msg)

    def create_subscriber_callback(self, event):
        for testblock in self.test.testblocks:
            for topic in self.get_topics_of_testblock(testblock.name):
                if topic not in list(self.subscribers.keys()):
                    subscriber = self.create_subscriber(topic)
                    if subscriber == None:
                        continue
                    # add new entry to subscribers dictionary
                    self.subscribers[topic] = {}
                    self.subscribers[topic]["testblocks"] = [testblock.name]
                    self.subscribers[topic]["subscriber"] = subscriber
                else:
                    self.subscribers[topic]["testblocks"].append(testblock.name)

    def diagnostics_callback(self, msg):
        self.diagnostics = msg

    def diagnostics_agg_callback(self, msg):
        self.diagnostics_agg = msg

    def filter_diagnostics_agg(self, diagnostics_agg):
        diagnostics_agg_error_only = DiagnosticArray()
        diagnostics_agg_error_only.header = diagnostics_agg.header
        for status in diagnostics_agg.status:
            if status.level != DiagnosticStatus.OK:
                diagnostics_agg_error_only.status.append(status)
        return diagnostics_agg_error_only

    def shutdown(self):
        rospy.loginfo("Shutdown ATF recorder and close bag file.")
        self.lock_write.acquire()
        self.bag.close()
        self.lock_write.release()

    def create_subscriber(self, topic):
        if not topic.startswith("/"):
            msg = "topic %s is not a global topic. Needs to start with '/'"%topic
            rospy.logerr(msg)
            raise ATFRecorderError(msg)
        try:
            if topic == "/tf" or topic == "/tf_static":
                msg_class = TFMessage
            else:
                msg_class, _, _ = rostopic.get_topic_class(topic)
            subscriber = rospy.Subscriber(topic, msg_class, self.global_topic_callback, callback_args=topic)
            rospy.logdebug("created subsriber for topic %s", topic)
        except Exception as e:
            msg = "Error while adding a subscriber for %s: %s."%(topic, e)
            rospy.logdebug(msg)
            #raise ATFRecorderError(msg)
            return None
        return subscriber

    def record_status(self, status):
        #print "status", status
        self.bag_file_writer.write_to_bagfile("/atf/status", status, status.stamp)

    def start_recording(self, testblock_name):
        self.lock.acquire()
        if testblock_name not in self.test.testblockset_config:
            raise ATFRecorderError("Testblock '%s' not in test config" % testblock_name)

        for topic in self.get_topics_of_testblock(testblock_name):
            if topic not in list(self.active_topics.keys()):
                self.active_topics[topic] = [testblock_name]
            else:
                self.active_topics[topic].append(testblock_name)

        #print ">>> start recording for %s, active_topics="%testblock_name, self.active_topics
        self.lock.release()

        # Send message to all recorder plugins
        #print "self.recorder_plugin_list=", self.recorder_plugin_list
        for recorder_plugin in self.recorder_plugin_list:
            # filter the recorder plugins not needed for current trigger/testblock
            if recorder_plugin.name in list(self.test.testblockset_config[testblock_name].keys()):
                #rospy.loginfo("recorder plugin callback for testblock: '%s'", testblock_name)
                recorder_plugin.trigger_callback(testblock_name)

    def stop_recording(self, testblock_name):
        self.lock.acquire()
        if testblock_name not in self.test.testblockset_config:
            raise ATFRecorderError("Testblock '%s' not in test config" % testblock_name)

        for topic in self.get_topics_of_testblock(testblock_name):
            #print "self.active_topics", self.active_topics
            if topic in list(self.active_topics.keys()):
                self.active_topics[topic].remove(testblock_name)
            #print "self.active_topics of topic '%s'"%topic, self.active_topics[topic]
            if len(self.active_topics[topic]) == 0:
                self.active_topics.pop(topic)

        #print "<<< stop recording for %s, active_topics="%testblock_name, self.active_topics
        self.lock.release()

    def get_topics_of_testblock(self, testblock_name):
        topics = []
        
        # topics from testblockset_config
        testblock_data = self.test.testblockset_config[testblock_name]
        #print "testblock_data=", testblock_data
#        for metric, metric_data in testblock_data.items():
#            #print "metric=", metric
#            #print "metric_data=", metric_data
#            for entry in metric_data:
#                # read all "topic" entries
#                if "topic" in entry:
#                    topic = entry["topic"]
#                    if topic not in topics:
#                        topics.append(topic)
#                        #print "topics==============", topics

        # ask each metric about its topics
        for testblock in self.test.testblocks:
            #print "testblock.name =", testblock.name
            if testblock.name == testblock_name:
                for metric_handle in testblock.metric_handles:
                    #print "metric_handle=", metric_handle
                    for topic in metric_handle.get_topics():
                        #print "topic=", topic
                        if topic not in topics:
                            topics.append(topic)
                    #print "  topics for metric %s of testblock %s ="%(str(metric_handle), testblock.name), topics
            else:
                #print "testblock names do not match %s %s"%(testblock.name, testblock_name)
                continue
        #print "topics of testblock %s ="%testblock_name, topics

        # fix global topic prefix
        topics_global = []
        for topic in topics:
            if topic.startswith("/"): # we need to use global topics because rostopic.get_topic_class(topic) can not handle non-global topics
                topics_global.append(topic)
            else:
                topics_global.append("/" + topic)
        #print "topics_global of testblock %s ="%testblock_name, topics_global
        return topics_global


    
    @staticmethod
    def load_data(filename):
        with open(filename, 'r') as stream:
            doc = yaml.safe_load(stream)
        if doc == None:
            doc = {}
        return doc

    def global_topic_callback(self, msg, name):
        # catch /tf_static messages because they are latched and would only be published once at the beginning but not at the beginning of each testblock
        # TODO generalize for other latched topics
        if name == "/tf_static":
            #check if message is already in list of messages
            for transform in msg.transforms:
                if not self.is_transform_in_tf_message(transform, self.tf_static_message):
                    self.tf_static_message.transforms.append(transform)
                    #rospy.loginfo("added to self.tf_static_message.transforms. len = %d", len(self.tf_static_message.transforms))
            return # this prevents TF_REPEATED_DATA
        
        if name in list(self.active_topics.keys()):
            self.bag_file_writer.write_to_bagfile(name, msg, rospy.Time.now())

    def is_transform_in_tf_message(self, transform, tf_message):
        for t in tf_message.transforms:

            if t.header.frame_id == transform.header.frame_id and t.child_frame_id == transform.child_frame_id:
                #rospy.logerr("transform is in tf_message. frame_if=%s, child_frame_id=%s", t.header.frame_id, t.child_frame_id)
                return True
        #rospy.loginfo("transform not in tf_message: %s --> %s", transform.header.frame_id, transform.child_frame_id)
        return False

    def tf_static_timer_callback(self, event):
        # republish latched /tf_static messages to /tf_static again
        if "/tf_static" in list(self.active_topics.keys()):
            for transform in self.tf_static_message.transforms:
                transform.header.stamp = rospy.Time.now()
            self.bag_file_writer.write_to_bagfile("/tf_static", self.tf_static_message, rospy.Time.now())
