#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
NORI_NAMESPACE_BEGIN
class DepthIntegrator : public Integrator
{
public:
    DepthIntegrator(const PropertyList &props)
    {
        /* No parameters this time */
    }
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Color3f Lo(0.);
        // Find the surface that is visible in the requested direction
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return scene->getBackground(ray);
        // Get all lights in the scene
        const std::vector<Emitter *> lights = scene->getLights();
        // Let's iterate over all emitters
        for (unsigned int i = 0; i < lights.size(); ++i)
        {
            // calculate depth
            float depth = 1 / its.t;

            Lo += depth;
        }
        return Lo;
    }
    std::string toString() const
    {
        return "Depth Integrator []";
    }
};
NORI_REGISTER_CLASS(DepthIntegrator, "depth");
NORI_NAMESPACE_END