#!/usr/bin/python
import rospy

import atf_core
from atf_msgs.msg import MetricResult
from atf_test_tools import PublishTf

class Application:
    def __init__(self):
        self.atf = atf_core.ATF()
        self.ptf = PublishTf()

    def execute(self):

        # small testblock (circle r=1, time=3)
        self.atf.start("testblock_small")
        # FIXME: due to timing problem the first tf message is sometimes omitted
        #        so next line (pub_zero) is used as a workaround
        self.ptf.pub_zero(doSleep=True)
        self.ptf.pub_circ(radius=1, time=3)

        # user result
        metric_result = MetricResult()
        metric_result.data.data = 0.8
        metric_result.groundtruth.result = True
        metric_result.groundtruth.error_message = "all ok in application of atf_test"
        self.atf.stop("testblock_small", metric_result)

        # large testblock (circle r=2, time=5)
        self.atf.start("testblock_large")
        self.ptf.pub_circ(radius=2, time=5)

        # user result
        metric_result = MetricResult()
        metric_result.data.data = 0.7
        self.atf.stop("testblock_large", metric_result)

        # shutdown atf
        self.atf.shutdown()

if __name__ == '__main__':
    rospy.init_node('test_app')
    app = Application()
    app.execute()
