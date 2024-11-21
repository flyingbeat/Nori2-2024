#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN
class PathTracingMIS : public Integrator
{
public:
    PathTracingMIS(const PropertyList &props)
    {
        /* No parameters this time */
    }

    /*
     * REFERENCES: Task description in assignment
     */
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        
    }

    std::string toString() const
    {
        return "Path Tracing with Multiple Importance Sampling []";
    }
};
NORI_REGISTER_CLASS(PathTracingMIS, "path_mis");
NORI_NAMESPACE_END
