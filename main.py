import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import theBattleground
import theRobot
import math
import matplotlib.pyplot as plt
import numpy as np

class Simulation():
    def __init__(self):

        self.Kp = 1
        self.Ki = 0.7
        self.Kd = 1

        self.maxspeed = 3
        self.TimestepCtr = 0

        self.errorArray = []
        self.errorIntArray = []
        self.errorDerArray = []
        self.outputArray = []
        self.angleArray = []

        self.mysystem = chrono.ChSystemNSC()
        self.ground = theBattleground.theBattleground(self.mysystem)
        self.robot = theRobot.theRobot(self.mysystem)
        self.createApplication()
        self.run()

    def createApplication(self):
        #  Create an Irrlicht application to visualize the system

        self.myapplication = chronoirr.ChIrrApp(self.mysystem, 'PyChrono example', chronoirr.dimension2du(1024,768))

        self.myapplication.AddTypicalSky()
        self.myapplication.AddTypicalLogo()
        self.myapplication.AddTypicalCamera(chronoirr.vector3df(0.6,0.6,0.8))
        self.myapplication.AddLightWithShadow(chronoirr.vector3df(2,4,2),    # point
                                         chronoirr.vector3df(0,0,0),    # aimpoint
                                         9,                 # radius (power)
                                         1,9,               # near, far
                                         30)                # angle of FOV

                 # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
                             # in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
                             # If you need a finer control on which item really needs a visualization proxy in
                             # Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

        self.myapplication.AssetBindAll();

                             # ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
                             # that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

        self.myapplication.AssetUpdateAll();
        self.myapplication.AddShadowAll();
        self.myapplication.SetShowInfos(True)

    def doMove(self):
        #print("z axis: " + str(self.robot.mbody1.GetRot().Q_to_Euler123().z*90 ))

        try:
            self.oldErrorFunc = self.errorFunc
        except:
            self.oldErrorFunc = 0

        self.errorFunc = ((self.robot.mbody1.GetRot().Q_to_Rotv().z) * 57.2957795) - 180


        self.errorFuncDerivative = self.errorFunc - self.oldErrorFunc

        try:
            self.errorFuncIntegral += self.errorFunc
        except:
            self.errorFuncIntegral = self.errorFunc

        self.output = (self.Kp * self.errorFunc) + (self.Ki * self.errorFuncIntegral) + (self.Kd * self.errorFuncDerivative)

        print("")
        print("no." + str(len(self.errorArray)) + "       error: " + str(self.errorFunc) + "       errorInt: " + str(self.errorFuncIntegral) + "         errorDer: " + str(self.errorFuncDerivative) + "          out :" + str(self.output))
        print("")

        if self.output < 0:
            self.robot.motor_R.SetMotorFunction(chrono.ChFunction_Const(self.maxspeed))
            self.robot.motor_L.SetMotorFunction(chrono.ChFunction_Const(self.maxspeed))
            print("fall back" + str(self.robot.motor_R.GetMotorFunction()))

        elif self.output > 0:
            self.robot.motor_R.SetMotorFunction(chrono.ChFunction_Const(-self.maxspeed))
            self.robot.motor_L.SetMotorFunction(chrono.ChFunction_Const(-self.maxspeed))
            print("fall forward")

        else:
            self.robot.motor_R.SetMotorFunction(chrono.ChFunction_Const(0))
            self.robot.motor_L.SetMotorFunction(chrono.ChFunction_Const(0))


        self.errorArray.append(self.errorFunc)
        self.errorIntArray.append(self.errorFuncIntegral)
        self.errorDerArray.append(self.errorFuncDerivative)
        self.outputArray.append(self.output)


    def run(self):
        #Timestep is 1ms
        self.myapplication.SetTimestep(0.1)
        self.myapplication.SetTryRealtime(True)

        while(self.myapplication.GetDevice().run()):
            #RoboResponsetime = Timestep * x milliseconds (eg. 10ms here)
            if self.TimestepCtr > 100:
                self.doMove();
                self.TimestepCtr = 0
            self.TimestepCtr += self.TimestepCtr + 1


            self.myapplication.BeginScene()
            self.myapplication.DrawAll()
            self.myapplication.DoStep()
            self.myapplication.EndScene()

        t = np.arange(0., len(self.errorArray), 1)
        plt.plot(t,self.outputArray, t, self.errorArray)
        plt.ylabel('some numbers')
        plt.show()

Simulation()
