#!/usr/bin/env python
import rospy
import rosgraph
import rosservice
import socket
import copy 
from atf_recorder import BagfileWriter
from rosapi.srv import Nodes, Topics, Publishers, Subscribers
from atf_msgs.msg import Api, NodeApi, InterfaceItem

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
            service_types = self.get_service_types(services)
        except socket.error: 
            raise ROSNodeIOException("Unable to communicate with master!") 

        #print "publishers=", publishers
        #print "subscribers=", subscribers
        #print "services=", services
        #print "topic_types=", topic_types
        #print "service_types=", service_types

        api_dict = {}
        self.add_api(api_dict, "publishers", publishers, topic_types)
        self.add_api(api_dict, "subscribers", subscribers, topic_types)
        self.add_api(api_dict, "services", services, service_types)
        #TODO actions
        
        #print "api_dict=\n", api_dict
        api = self.dict_to_msg(api_dict)
        #print "api=\n", api

        # write api to bagfile
        self.BfW.write_to_bagfile("/atf/" + msg.name + "/api", api, rospy.Time.now())

    def get_service_types(self, services):
        service_types = []
        for service in services:
            service_types.append([service[0], rosservice.get_service_type(service[0])])
        return service_types

    def add_api(self, api, api_descriptor, api_state, types):
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
                api_type = self.match_type(name, types)
                api[node][api_descriptor].append([name, api_type])
                #print "api=", api

    def match_type(self, name, types):
        #print "name=", name
        #print "types=", types
        for item in types:
            if item[0] == name:
                #print "type=", item[1]
                return item[1]
        return None

    def dict_to_msg(self, api_dict):
        api = Api()
        #print "api=", api
        # fill Api message
        for node, data in api_dict.items():
            #print ""
            #print "node=", node
            #print "data=", data
            node_api = NodeApi()
            node_api.name = node
            #print "node_api1=", node_api
            for api_descriptor, api_data in data.items():
                #print "api_descriptor=", api_descriptor
                #print "api_data=", api_data
                for item in api_data:
                    #print "item=", item
                    interface_item = InterfaceItem()
                    interface_item.name = item[0]
                    interface_item.type = item[1]
                    getattr(node_api.interface, api_descriptor).append(interface_item)
                #print "node_api2=", node_api
            #if "subscribers" in data:
            #    node_api.interface.subscribers = data["subscribers"]
            #if "services" in data:
            #    node_api.interface.services = data["services"]
            #TODO actions
            api.nodes.append(node_api)
        #print "api=\n", api
        return api
