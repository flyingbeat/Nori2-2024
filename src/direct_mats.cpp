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

        // indirect illumination
        Vector3f wi = its.toWorld(bsdfRecord.wo);
        Ray3f sampledRay(its.p, wi);
        Intersection shadowIts;
        if (scene->rayIntersect(sampledRay, shadowIts))
        {
            if (shadowIts.mesh->isEmitter())
            {
                const Emitter *emitter = shadowIts.mesh->getEmitter();
                EmitterQueryRecord emitterRecord(its.mesh->getEmitter(), sampledRay.o, shadowIts.p, shadowIts.shFrame.n, shadowIts.uv);

                Color3f Le = emitter->eval(emitterRecord);

                // fr is already multiplied by cosTheta and divided by pdf
                Lo += Le * fr;
            }
        }
        else
        {
            Color3f Le = scene->getBackground(sampledRay);
            Lo += Le * fr;
        }

        return Lo;
    }

    std::string toString() const
    {
        return "Direct Material Sampling []";
    }
};
NORI_REGISTER_CLASS(DirectMaterialSampling, "direct_mats");
NORI_NAMESPACE_END
