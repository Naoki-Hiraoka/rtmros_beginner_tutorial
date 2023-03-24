#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

############################################################

# python standard modules
import sys

# rtm modules
import OpenRTM_aist
import RTC

module_spec = ["implementation_id", "Publisher",
               "type_name",         "Publisher",
               "description",       "sample component",
               "version",           "1.0",
               "vendor",            "Naoki-Hiraoka",
               "category",          "example",
               "activity_type",     "DataFlowComponent",
               "max_instance",      "10",
               "language",          "Python",
               "lang_type",         "script",
               ""]

#
# Component
#
class Publisher(OpenRTM_aist.DataFlowComponentBase):
    def __init__ (self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        self.q = RTC.TimedDoubleSeq(RTC.Time(0,0),[])
        self.chatter = OpenRTM_aist.OutPort("chatter", self.q)
        return

    def onInitialize(self):
        print('onInitialize')
        self.registerOutPort("chatter", self.chatter)
        return RTC.RTC_OK

    def onActivated(self, ec_id):
        print('onActivated')
        return RTC.RTC_OK

    def onDeactivated(self, ec_id):
        print('onDeactivated')
        return RTC.RTC_OK

    def onExecute(self, ec_id):
        print('onExecute')
        self.q.data = [1.0, 2.0, 3.0]
        self.chatter.write()
        return RTC.RTC_OK


def PublisherInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=module_spec)
    manager.registerFactory(profile,
                            Publisher,
                            OpenRTM_aist.Delete)
    comp = manager.createComponent("Publisher")

if __name__ == '__main__':
    mgr = OpenRTM_aist.Manager.init(sys.argv)
    mgr.setModuleInitProc(PublisherInit)
    mgr.activateManager()
    mgr.runManager()
