#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN
class PathTracingNEE : public Integrator
{
public:
    PathTracingNEE(const PropertyList &props)
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
        if (sampler->next1D() > q)
        {
            return Lo;
        }

        // emitter sampling
        float pdfEmitter;
        // Get random light in the scene
        const Emitter *em = scene->sampleEmitter(sampler->next1D(), pdfEmitter);

        // Here we sample the point sources, getting its radiance
        // and direction.
        EmitterQueryRecord emitterRecord(its.p);
        Color3f Le = em->sample(emitterRecord, sampler->next2D(), 0.0f);
        Ray3f shadowRay(its.p, emitterRecord.wi);
        Intersection shadowIts;
        if (!(scene->rayIntersect(shadowRay, shadowIts) && shadowIts.t <= emitterRecord.dist) && bsdfRecord.measure == EDiscrete)
        {
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
        }

        Vector3f wi = its.toWorld(bsdfRecord.wo);
        Ray3f sampledRay(its.p, wi);
        Lo += Li(scene, sampler, sampledRay) * fr;

        return Lo;
    }

    std::string toString() const
    {
        return "Path Tracing with Next-Event Estimation []";
    }
};
NORI_REGISTER_CLASS(PathTracingNEE, "path_nee");
NORI_NAMESPACE_END
