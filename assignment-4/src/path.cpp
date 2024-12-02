#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN
class PathTracing : public Integrator
{
public:
    PathTracing(const PropertyList &props)
    {
        /* No parameters this time */
    }

    /*
     * REFERENCES: Task description in assignment
     */
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Color3f Lo(0.);
        // Find the surface that is visible in the requested direction
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return scene->getBackground(ray);

        // Direct illumination from emitter
        if (its.mesh->isEmitter())
        {
            EmitterQueryRecord emitterRecord(ray.o);
            emitterRecord.p = its.p;
            emitterRecord.wi = its.p - ray.o;
            emitterRecord.n = its.shFrame.n;
            return its.mesh->getEmitter()->eval(emitterRecord);
        }

        // sampling from intersection point
        const BSDF *bsdf = its.mesh->getBSDF();

        BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
        bsdfRecord.measure = ESolidAngle;

        Color3f fr = bsdf->sample(bsdfRecord, sampler->next2D());
        if (fr.isZero())
            // sampling failed
            return Lo;

        float q = 0.95f;
        // keep going
        if (sampler->next1D() <= q)
        {
            Vector3f wi = its.toWorld(bsdfRecord.wo);
            Ray3f sampledRay(its.p, wi);
            Intersection shadowIts;
            Lo += Li(scene, sampler, sampledRay) * fr / q;
        }

        return Lo;
    }

    std::string toString() const
    {
        return "Path Tracing []";
    }
};
NORI_REGISTER_CLASS(PathTracing, "path");
NORI_NAMESPACE_END
