#!/usr/bin/env python

import rospy
import copy

import world_canvas_client

from yocs_msgs.msg import *
from world_canvas_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('test_annotation_collection')
    topic_name  = rospy.get_param('~topic_name', 'annotations')
    topic_type  = rospy.get_param('~topic_type', None)
    pub_as_list = rospy.get_param('~pub_as_list', False)
    namespace   = rospy.get_param('~srv_namespace', '')
    world    = rospy.get_param('~world')
    uuids    = rospy.get_param('~uuids', [])
    names    = rospy.get_param('~names', [])
    types    = rospy.get_param('~types', [])
    keywords = rospy.get_param('~keywords', [])
    related  = rospy.get_param('~relationships', [])

    ac = world_canvas_client.AnnotationCollection(world, uuids, names, types, keywords, related, namespace)
    ac.load_data()
    walls = ac.get_data_of_type(yocs_msgs.msg.Wall)
    columns = ac.get_data_of_type(yocs_msgs.msg.Column)

    # Publish annotations' visual markers on client side
    ac.publish_markers('annotation_markers')

    # Publish annotations on client side
    ac.publish(topic_name + '_client', topic_type, False, pub_as_list)

    # Request server to also publish the same annotations
    ac.publish(topic_name,             topic_type, True,  pub_as_list)
    
    rospy.loginfo("Done")
    rospy.spin()
