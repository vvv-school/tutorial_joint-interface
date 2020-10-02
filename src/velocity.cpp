// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// A tutorial on how to use the Gaze Interface.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cstdlib>
#include <mutex>
#include <condition_variable>
#include <cmath>
#include <limits>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Time.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;


class CtrlModule: public RFModule
{
protected:
    PolyDriver        clienJoint;
    IControlLimits   *ilim;
    IEncoders        *ienc;
    IControlMode     *imod;
    IVelocityControl *ivel;

    RpcServer rpc;

	mutex mtx_done;
	condition_variable cv_done;

    int joint;
    double target;
    bool control;

    void go()
    {
        // retrieve joint bounds
        double min,max,range;
        ilim->getLimits(joint,&min,&max);
        range=max-min;

        // retrieve current joint position
        double enc;
        ienc->getEncoder(joint,&enc);

        // select target
        if (fabs(enc-min)<fabs(enc-max))
            target=max-0.1*range;
        else
            target=min+0.1*range;

        // set control mode
        imod->setControlMode(joint,VOCAB_CM_VELOCITY);

        // remove any possible limitation on the acceleration
        ivel->setRefAcceleration(joint,numeric_limits<double>::max());
        
        // start the control
        yInfo()<<"Yielding new target: "<<target<<" [deg]";
        control=true;

        // wait until we're done
		unique_lock<mutex> lck(mtx_done);
		cv_done.wait(lck);
		yInfo()<<"We're done";
    }

public:
    virtual bool configure(ResourceFinder &rf)
    {
        // open a client interface to connect to the joint controller
        Property optJoint;
        optJoint.put("device","remote_controlboard");
        optJoint.put("remote","/icubSim/left_arm");
        optJoint.put("local","/velocity/left_arm");

        if (!clienJoint.open(optJoint))
        {
            yError()<<"Unable to connect to /icubSim/left_arm";
            return false;
        }

        // open views
        bool ok=true;
        ok=ok && clienJoint.view(ilim);
        ok=ok && clienJoint.view(ienc);
        ok=ok && clienJoint.view(imod);
        ok=ok && clienJoint.view(ivel);

        if (!ok)
        {
            yError()<<"Unable to open views";
            return false;
        }

        // elbow
        joint=3;

        // start with control disabled
        control=false;

        // target = current joint position
        ienc->getEncoder(joint,&target);

        // open rpc port
        rpc.open("/velocity");

        // attach the callback respond()
        attach(rpc);

        return true;
    }

    virtual bool close()
    {
        rpc.close();
        clienJoint.close();

        return true;
    }

    virtual bool respond(const Bottle &cmd, Bottle &reply)
    {
        if (cmd.get(0).asString()=="go")
        {
            go();
            reply.addString("ack");
        }
        else if (cmd.get(0).asString()=="enc")
        {
            double enc;
            ienc->getEncoder(joint,&enc);
            reply.addString("ack");
            reply.addDouble(enc);
        }
        else
            reply.addString("nack");

        return true;
    }

    virtual double getPeriod()
    {
        // we set up the control rate here in [s]
        return 0.02;
    }

    virtual bool updateModule()
    {
        if (control)
        {
            // retrieve current joint position
            double enc;
            ienc->getEncoder(joint,&enc);

            // perform P control
            double Kp=2.0;
            double error=target-enc;
            ivel->velocityMove(joint,Kp*error);

            // notify we're done
            if (fabs(target-enc)<1.0)
            {
                control=false;
                ivel->velocityMove(joint,0.0);
				cv_done.notify_all();
            }
        }

        return true;
    }
};



int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    CtrlModule mod;
    return mod.runModule(rf);
}
