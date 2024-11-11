#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN
class DirectMaterialSampling : public Integrator
{
public:
    DirectMaterialSampling(const PropertyList &props)
    {
        /* No parameters this time */
    }
    /*
     * REFERENCES: Task description in assignment 
     */
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
      // Implement here
    }

    std::string toString() const
    {
        return "Direct Material Sampling []";
    }
};
NORI_REGISTER_CLASS(DirectMaterialSampling, "direct_mats");
NORI_NAMESPACE_END
