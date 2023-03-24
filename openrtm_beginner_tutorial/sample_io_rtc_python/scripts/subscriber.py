#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

############################################################

# python standard modules
import sys

# rtm modules
import OpenRTM_aist
import RTC

module_spec = ["implementation_id", "Subscriber",
               "type_name",         "Subscriber",
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
class Subscriber(OpenRTM_aist.DataFlowComponentBase):
    def __init__ (self, manager):
        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        self.q = RTC.TimedDoubleSeq(RTC.Time(0,0),[])
        self.listener = OpenRTM_aist.InPort("listener", self.q)
        return

    def onInitialize(self):
        print('onInitialize')
        self.registerInPort("listener", self.listener)
        return RTC.RTC_OK

    def onActivated(self, ec_id):
        print('onActivated')
        return RTC.RTC_OK

    def onDeactivated(self, ec_id):
        print('onDeactivated')
        return RTC.RTC_OK

    def onExecute(self, ec_id):
        print('onExecute')
        if self.listener.isNew():
            self.q = self.listener.read()
            print self.q.data
        return RTC.RTC_OK


def SubscriberInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=module_spec)
    manager.registerFactory(profile,
                            Subscriber,
                            OpenRTM_aist.Delete)
    comp = manager.createComponent("Subscriber")

if __name__ == '__main__':
    mgr = OpenRTM_aist.Manager.init(sys.argv)
    mgr.setModuleInitProc(SubscriberInit)
    mgr.activateManager()
    mgr.runManager()
