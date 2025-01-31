# please put this file in the directory where you excute choreonoid

import cnoid.Base
import cnoid.BodyPlugin
def addExternalForce():
 robotname = "JVRC1"
 linkname = "R_WRIST_R"
 pos = [0,0,0]
 force = [0,0,20]
 tm = 1
 robotItem = cnoid.Base.RootItem.instance.findItem(robotname)
 simulatorItem = cnoid.BodyPlugin.SimulatorItem.findActiveSimulatorItemFor(robotItem)
 pushingLink = robotItem.body.link(linkname)
 if pushingLink:
  simulatorItem.setExternalForce(robotItem, pushingLink, pos, force, tm)