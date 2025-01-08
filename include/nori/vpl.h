#pragma once

#include <nori/color.h>
#include <nori/vector.h>

namespace nori
{

    enum Type
    {
        EEmitterVPL,
        ESurfaceVPL
    }; // Direct or indirect VPL
    struct VPL
    {
        Type type;
        Point3f position; // Position in the scene
        Vector3f normal;  // Surface normal at the position
        Color3f flux;     // Light flux carried by this VPL

        VPL(const Type type, const Vector3f &pos, const Vector3f &norm, const Color3f &flux)
            : type(type), position(pos), normal(norm), flux(flux) {}
    };

}
