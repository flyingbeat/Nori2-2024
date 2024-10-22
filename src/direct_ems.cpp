#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN
class DirectEmitterSampling : public Integrator
{
public:
    DirectEmitterSampling(const PropertyList &props)
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

        EmitterQueryRecord emitterRecord(its.p);

        // Check if the intersected material is an emitter
        if (its.mesh->isEmitter())
        {
            EmitterQueryRecord emitterRecord(its.mesh->getEmitter(), ray.o, its.p, its.shFrame.n, its.uv);
            return its.mesh->getEmitter()->eval(emitterRecord);
        }

        float pdfEmitter;
        // Get random light in the scene
        const Emitter *em = scene->sampleEmitter(sampler->next1D(), pdfEmitter);

        // Here we sample the point sources, getting its radiance
        // and direction.
        Color3f Le = em->sample(emitterRecord, sampler->next2D(), 0.0f);

        // Here perform a visibility query, to check whether the light
        // source "em" is visible from the intersection point.
        // For that, we create a ray object (shadow ray),
        // and compute the intersection
        // V function in equation term
        Ray3f shadowRay(its.p, emitterRecord.wi);
        Intersection shadowIts;
        if (scene->rayIntersect(shadowRay, shadowIts) && shadowIts.t <= emitterRecord.dist)
            return Lo;

        // Finally, we evaluate the BSDF. For that, we need to build
        // a BSDFQueryRecord from the outgoing direction (the direction
        // of the primary ray, in ray.d), and the incoming direction
        // (the direction to the light source, in emitterRecord.wi).
        // Note that: a) the BSDF assumes directions in the local frame
        // of reference; and b) that both the incoming and outgoing
        // directions are assumed to start from the intersection point.
        BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d),
                                   its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);

        Color3f fr = its.mesh->getBSDF()->eval(bsdfRecord);

        // pΩ(x, x(k) l ) is the product of the pdf of choosing the light source and the pdf of x(k) at the light source.
        float pOmega = pdfEmitter * em->pdf(emitterRecord);
        // calculating the cosin term
        float cosTheta = its.shFrame.n.dot(emitterRecord.wi);

        Lo += (Le * fr * cosTheta) / pOmega;
        return Lo;
    }

    std::string toString() const
    {
        return "Direct Emitter Sampling []";
    }
};
NORI_REGISTER_CLASS(DirectEmitterSampling, "direct_ems");
NORI_NAMESPACE_END