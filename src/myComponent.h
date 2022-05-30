#ifndef MYCOMPONENT_H
#define MYCOMPONENT_H

#include "mbed.h"
#include <cstdint>
#include <string>
#include <vector>
#include "HydraComponent.h"
#include "HydraVariable.h"
#include "Wheel.h"



class myComponent: public HydraComponent{
    public:
        myComponent();

        // Specific Hydra functions
        virtual void __on__(std::string event_name, std::map<std::string, HydraData*> data);
        virtual void __loop__();

        // Component function
        HydraData* move(std::vector<HydraData*> parameters);
        HydraData* stop(std::vector<HydraData*> parameters);

    private:
        Wheel *_wheel;

    protected:
        float   _des_spd;
        int     _type_move;
        int _toto;
        
};
#endif