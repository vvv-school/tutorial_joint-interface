// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// A tutorial on how to use the Gaze Interface.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Time.h>
#include <yarp/os/Event.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>

using namespace yarp::os;
using namespace yarp::dev;


class CtrlModule: public RFModule
{
protected:
    PolyDriver       clienJoint;
    IControlLimits2 *ilim;
    IEncoders       *ienc;
    IControlMode2   *imod;
    IPositionDirect *iposd;

    RpcServer rpc;

    Event done;
    int joint;
    double target;
    bool control;
    double enc0;
    double t0;

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
        imod->setControlMode(joint,VOCAB_CM_POSITION_DIRECT);

        // start the control
        yInfo()<<"Yielding new target: "<<target<<" [deg]";
        t0=Time::now();
        enc0=enc;
        control=true;

        // wait until we're done
        done.reset();
        done.wait();
        yInfo()<<"We're done";
    }

public:
    virtual bool configure(ResourceFinder &rf)
    {
        // open a client interface to connect to the joint controller
        Property optJoint;
        optJoint.put("device","remote_controlboard");
        optJoint.put("remote","/icubSim/left_arm");
        optJoint.put("local","/positiondirect/left_arm");

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
        ok=ok && clienJoint.view(iposd);

        if (!ok)
        {
            yError()<<"Unable to open views";
            return false;
        }

        // elbow
        joint=3;

        // start with control disabled
        control=false;

        // open rpc port
        rpc.open("/positiondirect");

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
            double T=5.0;   // time period in [s]
            double dt=Time::now()-t0;

            // time ratios
            double r=dt/T;
            double r3=r*r*r;
            double r4=r3*r;
            double r5=r4*r;

            // minimum-jerk formula
            double ref=enc0+(target-enc0)*(10.0*r3-15.0*r4+6.0*r5);
            iposd->setPosition(joint,ref);

            // notify we're done
            if (dt>T)
            {
                control=false;
                done.signal();
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
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    CtrlModule mod;
    return mod.runModule(rf);
}
