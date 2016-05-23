#!/usr/bin/env python
import rospy
import rosgraph
import socket
import copy 
from atf_recorder import BagfileWriter
from rosapi.srv import Nodes, Topics, Publishers, Subscribers
from atf_msgs.msg import Api, NodeApi

class RecordInterface:
    def __init__(self, write_lock, bag_file):
        self.rosapi_service_nodes = rospy.ServiceProxy('/rosapi/nodes', Nodes)
        self.rosapi_service_topics = rospy.ServiceProxy('/rosapi/topics', Topics)
        self.rosapi_service_publishers = rospy.ServiceProxy('/rosapi/publishers', Publishers)
        self.rosapi_service_subscribers = rospy.ServiceProxy('/rosapi/subscribers', Subscribers)
        
        self.master = rosgraph.Master("/rosnode")

        self.BfW = BagfileWriter(bag_file, write_lock)

    def trigger_callback(self, msg):
        #print "msg=", msg

        try: 
            publishers, subscribers, services = self.master.getSystemState() 
            #pub_topics = self.master.getPublishedTopics('/subscriber1')
            topic_types = self.master.getTopicTypes()  
        except socket.error: 
            raise ROSNodeIOException("Unable to communicate with master!") 

        #print "publishers=", publishers
        #print "subscribers=", subscribers
        #print "services=", services
        #print "pub_topics=", pub_topics
        #print "topic_types=", topic_types

        api_dict = {}
        self.add_api(api_dict, "publishers", publishers)
        self.add_api(api_dict, "subscribers", subscribers)
        self.add_api(api_dict, "services", services)
        #TODO actions
        #print "api_dict=\n", api_dict
        
        api = self.dict_to_msg(api_dict)

        # write api to bagfile
        self.BfW.write_to_bagfile("/atf/" + msg.name + "/api", api, rospy.Time.now())

    def dict_to_msg(self, api_dict):
        api = Api()
        #print "api=", api
        # fill Api message
        for node, data in api_dict.items():
            #print "node=", node
            #print "data=", data
            node_api = NodeApi()
            node_api.name = node
            if "publishers" in data:
                node_api.interface.publishers = data["publishers"]
            if "subscribers" in data:
                node_api.interface.subscribers = data["subscribers"]
            if "services" in data:
                node_api.interface.services = data["services"]
            #TODO actions
            api.nodes.append(node_api)
        #print "api=\n", api
        return api
    
    def add_api(self, api, api_descriptor, api_state):
        for name, nodes in api_state:
            #print "name=", name
            #print "nodes=", nodes
            for node in nodes:
                #print "node=", node
                if not node in api:
                    #print "new"
                    api[node] = {}
                else:
                    #print "merge"
                    pass
                if not api_descriptor in api[node]:
                    api[node][api_descriptor] = []
                api[node][api_descriptor].append(name)
                #print "api=", api
