#pragma once

#include <nori/color.h>
#include <nori/vector.h>

namespace nori
{
    enum EVPLType
    {
        DIRECT,
        INDIRECT
    };

    struct VPL
    {
        EmitterQueryRecord pRec; // EmitterQueryRecord of the VPL
        Color3f flux;            // Light flux carried by this VPL
        EVPLType type;           // Type of VPL

        VPL(const EVPLType &type, const EmitterQueryRecord &pRec, const Color3f &flux)
            : type(type), pRec(pRec), flux(flux) {}
    };

}
