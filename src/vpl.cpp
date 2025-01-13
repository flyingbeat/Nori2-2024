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
        m_numVPLs = props.getInteger("num_vpls", 100);
        m_maxDepth = props.getInteger("max_depth", 2);
        m_showVPLs = props.getBoolean("show_vpls", false);
    }

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

        // Iterate over all VPLs to compute their contribution
        for (const VPL &vpl : m_vpls)
        {
            // Compute the vector from the shading point to the VPL
            Vector3f lightDir = (vpl.pRec.p - its.p).normalized();
            float distanceSquared = (vpl.pRec.p - its.p).squaredNorm();

            // Check visibility (shadow ray)
            Ray3f shadowRay(its.p, lightDir);
            Intersection shadowIts;
            if (scene->rayIntersect(shadowRay, shadowIts) && shadowIts.t * shadowIts.t <= (distanceSquared - (Epsilon * Epsilon)))
                continue; // Skip if the VPL is occluded

            // Highlight the VPLs in green
            if (m_showVPLs && (vpl.pRec.p - its.p).norm() <= 0.015)
            {
                return Color3f(0, 1, 0);
            }

            // Compute the BRDF at the shading point
            BSDFQueryRecord bsdfRec(its.toLocal(-ray.d), its.toLocal(lightDir), its.uv, ESolidAngle);
            Color3f bsdfValue = its.mesh->getBSDF()->eval(bsdfRec);

            // Compute the geometric term
            float cosTheta = std::max(0.0f, its.shFrame.n.dot(lightDir));
            float vplCosTheta = std::max(0.0f, vpl.pRec.n.dot(-lightDir));
            float geometryTerm = (cosTheta * vplCosTheta) / distanceSquared;

            //  Accumulate the VPL's contribution
            Lo += vpl.flux * bsdfValue * geometryTerm;
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
            Color3f Le = emitter->sample(pRec, sampler->next2D(), 0.0f);
            pRec.emitter = emitter;
            float pdfPoint = emitter->getMesh()->pdf(pRec.p);

            Color3f flux = (Le / (pdfPoint * pdfEmitter));

            //  Add the initial VPL based on the emitter
            VPL vpl(DIRECT, pRec, flux);
            m_vpls.push_back(vpl);

            // Trace a random walk for indirect VPLs
            generateIndirectVPLs(scene, sampler, vpl.flux, pRec, m_vpls);
        }
        // Normalize the VPLs' flux by the number of VPLs
        for (VPL &vpl : m_vpls)
        {
            vpl.flux /= (float)m_numVPLs;
        }
    }

    std::string toString() const
    {
        return tfm::format(
            "VPLIntegrator[\n"
            "  num_vpls = %s,\n"
            "  max_depth = %s,\n"
            "  show_vpls = %s\n"
            "]",
            m_numVPLs,
            m_maxDepth,
            m_showVPLs);
    }

private:
    void generateIndirectVPLs(const Scene *scene, std::unique_ptr<Sampler> &sampler,
                              const Color3f &flux, const EmitterQueryRecord &pRec, std::vector<VPL> &vpls)
    {
        // Sample a direction from the emitter
        Vector3f localDir = Warp::squareToCosineHemisphere(sampler->next2D());
        // float dPdf = Warp::squareToCosineHemispherePdf(localDir);
        Vector3f worldDir = Frame(pRec.n).toWorld(localDir);
        Ray3f ray(pRec.p, worldDir);
        Color3f weight = flux; // dPdf;
        int depth = 0;

        while (depth++ < m_maxDepth && !weight.isZero())
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

            // Update the weight with the sampled BSDF value and the PDF
            weight *= bsdfValue;

            // Create a new VPL at this intersection
            EmitterQueryRecord pRec;
            pRec.p = its.p;
            pRec.n = its.shFrame.n;
            VPL indirectVPL(INDIRECT, pRec, weight);
            vpls.push_back(indirectVPL);

            // Russian roulette termination
            float rrProbability = std::min(weight.maxCoeff(), 1.0f);
            if (sampler->next1D() > rrProbability)
                break; // Terminate the path

            // Scale weight to account for Russian roulette termination
            weight /= rrProbability;

            // Update the ray for the next iteration
            ray = Ray3f(its.p, its.toWorld(bRec.wo));
        }
    }

private:
    std::vector<VPL> m_vpls;
    int m_numVPLs;
    bool m_showVPLs;
    int m_maxDepth;
};

NORI_REGISTER_CLASS(VPLIntegrator, "vpl");
NORI_NAMESPACE_END