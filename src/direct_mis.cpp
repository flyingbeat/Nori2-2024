#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN
class DirectMIS : public Integrator
{
public:
    DirectMIS(const PropertyList &props)
    {
        /* No parameters this time */
    }

    /*
     * REFERENCES: Task description in assignment
     */
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Color3f Lo(0.);

        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return scene->getBackground(ray);

        EmitterQueryRecord emitterRecord(its.p);

        if (its.mesh->isEmitter())
        {
            EmitterQueryRecord emitterRecord(ray.o);
            emitterRecord.p = its.p;
            emitterRecord.wi = its.p - ray.o;
            emitterRecord.n = its.shFrame.n;
            return its.mesh->getEmitter()->eval(emitterRecord);
        }

        // light sampling
        float pdfEmitter;
        // Get random light in the scene
        const Emitter *em = scene->sampleEmitter(sampler->next1D(), pdfEmitter);

        // Here we sample the point sources, getting its radiance
        // and direction.
        Color3f LiEms = em->sample(emitterRecord, sampler->next2D(), 0.0f);

        // pΩ(x, x(k) l ) is the product of the pdf of choosing the light source and the pdf of x(k) at the light source.
        float p_emW_em = pdfEmitter * em->pdf(emitterRecord);

        // bsdf sampling
        const BSDF *bsdf = its.mesh->getBSDF();

        BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
        bsdfRecord.measure = ESolidAngle;

        Color3f frMats = bsdf->sample(bsdfRecord, sampler->next2D());
        float p_matW_mat = bsdf->pdf(bsdfRecord);

        // Here perform a visibility query, to check whether the light
        // source "em" is visible from the intersection point.
        // For that, we create a ray object (shadow ray),
        // and compute the intersection and check that the intersection is closer than the light source.
        // V function in equation term
        Ray3f shadowRay(its.p, emitterRecord.wi);
        Intersection shadowItsEms;
        if (!(scene->rayIntersect(shadowRay, shadowItsEms) && shadowItsEms.t <= emitterRecord.dist))
        {
            BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d),
                                       its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);

            float cosTheta = its.shFrame.n.dot(emitterRecord.wi);
            float p_mat_Wem = bsdf->pdf(bsdfRecord);
            Color3f frEms = its.mesh->getBSDF()->eval(bsdfRecord);
            float wem = p_emW_em / (p_emW_em + p_mat_Wem);

            Lo += wem * ((LiEms * frEms * cosTheta) / p_emW_em);
        }

        // Here perform a visibility query, to check whether the light
        // source "em" is visible from the intersection point.
        // For that, we create a ray object (shadow ray),
        // and compute the intersection
        Vector3f wi = its.toWorld(bsdfRecord.wo);
        Ray3f sampledRay(its.p, wi);
        Intersection shadowItsMats;
        if (scene->rayIntersect(sampledRay, shadowItsMats))
        {
            if (shadowItsMats.mesh->isEmitter())
            {
                const Emitter *emitter = shadowItsMats.mesh->getEmitter();
                EmitterQueryRecord emitterRecord(emitter, sampledRay.o, shadowItsMats.p, shadowItsMats.shFrame.n, shadowItsMats.uv);

                float p_emW_mat = emitter->pdf(emitterRecord);

                float wmat = p_matW_mat / (p_matW_mat + p_emW_mat);

                Color3f LiMat = emitter->eval(emitterRecord);

                Lo += wmat * LiMat * frMats;
            }
        }
        else
        {
            Color3f LiMat = scene->getBackground(sampledRay);
            Lo += LiMat * frMats;
        }

        return Lo;
    }

    std::string toString() const
    {
        return "Direct Multiple Importance Sampling []";
    }
};
NORI_REGISTER_CLASS(DirectMIS, "direct_mis");
NORI_NAMESPACE_END
