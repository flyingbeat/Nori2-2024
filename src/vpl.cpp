#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/vpl.h>
#include <nori/sampler.h>
#include <nori/warp.h>
#include <iostream>

NORI_NAMESPACE_BEGIN

class VPLIntegrator : public Integrator
{
public:
    VPLIntegrator(const PropertyList &props)
    {
        m_showVPLs = props.getBoolean("show_vpls", false);
        m_numVPLs = props.getInteger("num_vpls", 100);
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Color3f Lo(0.);
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return scene->getBackground(ray);

        if (its.mesh->isEmitter())
        {
            EmitterQueryRecord emitterRecord(ray.o);
            emitterRecord.p = its.p;
            emitterRecord.wi = its.p - ray.o;
            emitterRecord.n = its.shFrame.n;
            return its.mesh->getEmitter()->eval(emitterRecord);
        }

        // Iterate over all VPLs to compute their contribution
        for (const VPL &vpl : m_vpls)
        {
            // Compute the vector from the shading point to the VPL
            Vector3f lightDir = (vpl.position - its.p).normalized();
            float distanceSquared = (vpl.position - its.p).squaredNorm();

            // Check visibility (shadow ray)
            Ray3f shadowRay(its.p, lightDir);
            Intersection shadowIts;
            if (scene->rayIntersect(shadowRay, shadowIts) && shadowIts.t <= std::sqrt(distanceSquared))
                continue; // Skip if the VPL is occluded

            // Highlight the VPLs in green
            if (m_showVPLs && (vpl.position - its.p).norm() <= 0.01)
            {
                return Color3f(0, 1, 0);
            }
            // Compute the BRDF at the shading point
            BSDFQueryRecord bsdfRec(its.toLocal(-ray.d), its.toLocal(lightDir), its.uv, ESolidAngle);
            Color3f bsdfValue = its.mesh->getBSDF()->eval(bsdfRec);

            // Compute the geometric term
            float cosTheta = std::max(0.0f, its.shFrame.n.dot(lightDir));
            float vplCosTheta = std::max(0.0f, vpl.normal.dot(-lightDir));
            float geometryTerm = (cosTheta * vplCosTheta) / distanceSquared;

            //  Accumulate the VPL's contribution
            Lo += (bsdfValue * vpl.flux * geometryTerm);
        }

        return Lo;
    }

    void preprocess(const Scene *scene)
    {
        // Create a sampler for random sampling
        std::unique_ptr<Sampler> sampler = scene->getSampler()->clone();

        // Loop to generate the specified number of VPLs
        for (int i = 0; i < m_numVPLs; ++i)
        {
            float pdfEmitter;
            // Sample an emitter and generate a VPL
            const Emitter *emitter = scene->sampleEmitter(sampler->next1D(), pdfEmitter);
            if (!emitter)
                continue;

            // Sample a position on the emitter
            EmitterQueryRecord pRec;
            Color3f weight = emitter->sample(pRec, sampler->next2D(), 0.0f);

            // Sample a direction from the emitter
            EmitterQueryRecord dRec;
            emitter->sampleDirection(pRec, dRec, sampler->next2D());

            // Add the initial VPL based on the emitter
            Vector3f normal = (dRec.wi - pRec.p).normalized();
            VPL vpl(EEmitterVPL, pRec.p, normal, weight);
            m_vpls.push_back(vpl);

            // Trace a random walk for indirect VPLs
            generateIndirectVPLs(scene, sampler, vpl, dRec, m_vpls);
        }
        // normalize the VPLs
        for (VPL &vpl : m_vpls)
        {
            vpl.flux /= m_vpls.size();
        }
    }

    std::string toString() const
    {
        return tfm::format(
            "VPLIntegrator[\n"
            "  num_vpls = %s,\n"
            "  show_vpls = %s\n"
            "]",
            m_numVPLs,
            m_showVPLs);
        return "VPLIntegrator []";
    }

private:
    void generateIndirectVPLs(const Scene *scene, std::unique_ptr<Sampler> &sampler,
                              const VPL &vpl, const EmitterQueryRecord &dRec, std::vector<VPL> &vpls)
    {
        Ray3f ray(vpl.position, dRec.wi);
        Color3f weight = vpl.flux;
        int depth = 0;

        while (depth++ < 3 && !weight.isZero())
        {
            Intersection its;
            if (!scene->rayIntersect(ray, its))
                break;

            // Get the BSDF at the intersection point
            const BSDF *bsdf = its.mesh->getBSDF();
            if (!bsdf)
                break;

            // Sample the BSDF to find the next direction
            BSDFQueryRecord bRec(its.toLocal(-ray.d));
            Color3f bsdfValue = bsdf->sample(bRec, sampler->next2D());
            if (bsdfValue.isZero())
                break;

            // Create a new VPL at this intersection
            VPL indirectVPL(ESurfaceVPL, its.p, its.shFrame.n, weight * bsdfValue);
            vpls.push_back(indirectVPL);

            // Prepare for the next bounce
            weight *= bsdfValue;
            ray = Ray3f(its.p, its.toWorld(bRec.wo));

            /* Prevent light leaks due to the use of shading normals -- [Veach, p. 158] */
            float wiDotGeoN = its.geoFrame.n.dot(-ray.d);
            float woDotGeoN = its.geoFrame.n.dot(its.toWorld(bRec.wo));
            if (wiDotGeoN * Frame::cosTheta(bRec.wi) <= 0 ||
                woDotGeoN * Frame::cosTheta(bRec.wo) <= 0)
                break;
        }
    }

private:
    std::vector<VPL> m_vpls;
    int m_numVPLs;
    bool m_showVPLs;
};

NORI_REGISTER_CLASS(VPLIntegrator, "vpl");
NORI_NAMESPACE_END