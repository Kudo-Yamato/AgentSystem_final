#include <cnoid/SimpleController>
#include <cnoid/Camera>
#include <cnoid/Joystick>
using namespace cnoid;
class SimpleCarController : public SimpleController
{
    
    Link* basejoint;
    double q_ref;
    double q_prev;
    double dt;

    Link* wheels[12];

    Link* arms[4];
    double q_arm_ref[4];
    double q_arm_prev[4];
    double q_arm[4];
    double dq_arm[4];
    double dq_arm_ref[4];


    Camera* camera;

    Joystick joystick;

    bool prevButtonState;
    std::ostream* os;

    public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        basejoint = io->body()->link("BASE"); // BASE（回転）
        basejoint ->setActuationMode(Link::JointTorque);
        io->enableIO(basejoint);
        q_ref = q_prev = basejoint->q();

        wheels[0] = io->body()->link("FRONTRIGHT_WHEEL");
        wheels[1] = io->body()->link("FRONTLEFT_WHEEL");
        wheels[2] = io->body()->link("REARLEFT_WHEEL");
        wheels[3] = io->body()->link("REARRIGHT_WHEEL");
        wheels[4] = io->body()->link("FRONTRIGHT_WHEEL2");
        wheels[5] = io->body()->link("FRONTLEFT_WHEEL2");
        wheels[6] = io->body()->link("REARLEFT_WHEEL2");
        wheels[7] = io->body()->link("REARRIGHT_WHEEL2");
        wheels[8] = io->body()->link("FRONTRIGHT_WHEEL3");
        wheels[9] = io->body()->link("FRONTLEFT_WHEEL3");
        wheels[10] = io->body()->link("REARLEFT_WHEEL3");
        wheels[11] = io->body()->link("REARRIGHT_WHEEL3");

        arms[0] = io->body()->link("FRONTRIGHT_ARM");
        arms[1] = io->body()->link("FRONTLEFT_ARM");
        arms[2] = io->body()->link("REARLEFT_ARM");
        arms[3] = io->body()->link("REARRIGHT_ARM");
        
        for(int i = 0; i < 12; ++i){
            wheels[i]->setActuationMode(Link::JointVelocity);
            io->enableOutput(wheels[i]);
        }

        for(int i = 0; i < 4; ++i){
            arms[i]->setActuationMode(Link::JointTorque); 
            // arms[i]->setActuationMode(Link::JointAngle);
            io->enableIO(arms[i]);
        }

        for(int i = 0; i < 4; ++i){
            q_arm_ref[i] = q_arm_prev[i] = arms[i]->q();
        }

        dt = io->timeStep();
        return true;
    }

    virtual bool control() override
    {
        static const double P = 200.0;
        static const double D = 50.0;
        joystick.readCurrentState();
        double q = basejoint->q();
        double dq = (q - q_prev) / dt;
        double dq_ref = 0.0;
        double pos = joystick.getPosition(2);
        if(fabs(pos) > 0.25){
        double deltaq = 0.002 * pos;
        q_ref += deltaq;
        dq_ref = deltaq / dt;
        }
        basejoint->u() = P * (q_ref - q) + D * (dq_ref - dq);
        q_prev = q;

        double forward = -joystick.getPosition(5); // Axis 1: forward/backward
        double turn    =  joystick.getPosition(4); // Axis 0: left/right

        double baseSpeed = 10.0;
        double turnSpeed = 10.0;

        double leftSpeed  = baseSpeed * forward + turnSpeed * turn;
        double rightSpeed = baseSpeed * forward - turnSpeed * turn;

        wheels[0]->dq_target() = rightSpeed; // FR
        wheels[1]->dq_target() = leftSpeed;  // FL
        wheels[2]->dq_target() = leftSpeed;  // RL
        wheels[3]->dq_target() = rightSpeed; // RR
        wheels[4]->dq_target() = rightSpeed; // FR
        wheels[5]->dq_target() = leftSpeed;  // FL
        wheels[6]->dq_target() = leftSpeed;  // RL
        wheels[7]->dq_target() = rightSpeed; // RR
        wheels[8]->dq_target() = rightSpeed; // FR
        wheels[9]->dq_target() = leftSpeed;  // FL
        wheels[10]->dq_target() = leftSpeed;  // RL
        wheels[11]->dq_target() = rightSpeed; // RR

        // arms[0]->q_target() = 0.09; // FR
        // arms[1]->q_target() = 0.09; // FL
        // arms[2]->q_target() = -0.09; // RL
        // arms[3]->q_target() = -0.09; // RR

        for(int i = 0; i < 4; ++i){
            q_arm[i] = arms[i]->q();
            dq_arm[i] = (q_arm[i] - q_arm_prev[i]) / dt;
            dq_arm_ref[i] = 0.0;
        }
        double pos_armf = joystick.getPosition(3); // Axis 3: arm control
        if(fabs(pos_armf) > 0.25){
            double deltaq_arm = 0.002 * pos_armf;
            for(int i = 0; i < 2; ++i){
                q_arm_ref[i] += deltaq_arm;
                dq_arm_ref[i] = deltaq_arm / dt;
                arms[i]->u() = 50 * (q_arm_ref[i] - q_arm[i]) + 10 * (dq_arm_ref[i] - dq_arm[i]);
            }
        }
        else {
            // 入力がない場合、現在の角度を維持
            for(int i = 0; i < 2; ++i) {
                arms[i]->u() = 50 * (q_arm_ref[i] - q_arm[i]) + 10 * (0.0 - dq_arm[i]);
            }
        }

        double pos_armr = joystick.getPosition(1); // Axis 3: arm control
        if(fabs(pos_armr) > 0.25){
            double deltaq_arm = 0.002 * -pos_armr;
            for(int i = 2; i < 4; ++i){
                q_arm_ref[i] += deltaq_arm;
                dq_arm_ref[i] = deltaq_arm / dt;
                arms[i]->u() = 50 * (q_arm_ref[i] - q_arm[i]) + 10 * (dq_arm_ref[i] - dq_arm[i]);
            }
        }
        else {
            // 入力がない場合、現在の角度を維持
            for(int i = 2; i < 4; ++i) {
                arms[i]->u() = 50 * (q_arm_ref[i] - q_arm[i]) + 10 * (0.0 - dq_arm[i]);
            }
        }


        for(int i = 0; i < 4; ++i){
            q_arm_prev[i] = q_arm[i];
        }

        


        return true;
    }
};
CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SimpleCarController)