#include "mbed.h"
#include "myComponent.h"
#include "Wheel.h"


myComponent::myComponent():HydraComponent("myComponent", "myComponent/type")
{
    _wheel = new Wheel(D3, D9, D6, D5, D4);       //pwm, fwd, rev, EncA, EncB
    _wheel->StartRegule();
    // // Timer
    // _t = new Timer;
    // _t->start();
    _des_spd  = 0;

    addHydraFunction(new HydraFunction("move",    callback(this, &myComponent::move),       1, 10));
    addHydraFunction(new HydraFunction("stop",    callback(this, &myComponent::stop),       0, 11));
    addHydraVariable(new HydraVariable("TOTO", new HydraData(&_toto), 12));
}

void myComponent::__loop__()
{
//  if (_t->read_ms() > 100){
//         // printf("_spd = %f\t_u = %f\t_e = %f\t_com = %f\r\n", _spd, _u[2], _e[2], _com[2]);
//         // printf("_spd = %f\r\n", _spd);
//         _t->reset();
//         _t->start();
//     }
}

void myComponent::__on__(std::string event_name, std::map<std::string, HydraData *> data)
{
}

// Component function
HydraData* myComponent::move(std::vector<HydraData*> parameters)
{
    int param_1 = parameters[0]->get<int>();
    _wheel->SetSpeedRPM(param_1);
    return new HydraData("vitesse: " +to_string(param_1));
}

HydraData* myComponent::stop(std::vector<HydraData*> parameters){
    _des_spd = 0;
    _wheel->SetSpeedRPM(_des_spd);
    return NULL;
}
