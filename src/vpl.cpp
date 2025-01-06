#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/vpl.h>
#include <nori/sampler.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class VPLIntegrator : public Integrator
{
public:
    VPLIntegrator(const PropertyList &props)
    {
        // no parameters this time
        m_numVPLs = props.getInteger("num_vpls", 100);
    }

    // void preprocess(const Scene *scene)
    // {
    //     cout << "Preprocessing VPLs..." << endl;
    //     // Initialize the sampler
    //     Sampler *sampler = const_cast<Sampler *>(scene->getSampler());

    //     // Iterate to generate the desired number of VPLs
    //     for (int i = 0; i < m_numVPLs; ++i)
    //     {
    //         // Step 1: Sample a light source
    //         float pdfEmitter;
    //         // Get random light in the scene
    //         const Emitter *em = scene->sampleEmitter(sampler->next1D(), pdfEmitter);
    //         if (!em)
    //             continue; // If no light is sampled, skip this iteration

    //         // Sample a ray from the light source
    //         EmitterQueryRecord emitterRec;
    //         Color3f LiEms = em->sample(emitterRec, sampler->next2D(), 0.0f);

    //         // Step 2: Trace the ray into the scene
    //         Ray3f lightRay(emitterRec.p, emitterRec.wi);
    //         Intersection its;
    //         if (!scene->rayIntersect(lightRay, its))
    //             continue; // If no intersection, skip this iteration

    //         // Step 3: Create a VPL at the intersection point
    //         VPL vpl(its.p, its.shFrame.n, LiEms);

    //         // Add the VPL to the list
    //         m_vpls.push_back(vpl);
    //     }
    // }

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
            EmitterQueryRecord eRec;
            Color3f flux = emitter->sample(eRec, sampler->next2D(), 0.0f);
            flux /= pdfEmitter; // Normalize by emitter PDF
            if (flux.isZero())
                continue;

            // Add the initial VPL based on the emitter
            VPL vpl(EEmitterVPL, eRec.p, eRec.n, flux);
            m_vpls.push_back(vpl);

            // Trace a random walk for indirect VPLs
            generateIndirectVPLs(scene, sampler, vpl, m_vpls);
        }
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
    {
        Color3f Lo(0.);
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return scene->getBackground(ray);

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

            // Compute the BRDF at the shading point
            BSDFQueryRecord bsdfRec(its.toLocal(-ray.d), its.toLocal(lightDir), its.uv, ESolidAngle);
            Color3f bsdfValue = its.mesh->getBSDF()->eval(bsdfRec);

            // Compute the geometric term
            float cosTheta = std::max(0.0f, its.shFrame.n.dot(lightDir));
            float vplCosTheta = std::max(0.0f, vpl.normal.dot(-lightDir));
            float geometryTerm = (cosTheta * vplCosTheta) / distanceSquared;

            // Lo += its.shFrame.n.dot(vpl.normal) * its.mesh->getBSDF()->eval(bsdfRec);
            //  Accumulate the VPL's contribution
            Lo += bsdfValue * vpl.flux * geometryTerm / m_numVPLs;
        }

        return Lo;
    }

    std::string toString() const
    {
        return tfm::format(
            "VPLIntegrator[\n"
            "  num_vpls = %s,\n"
            "]",
            m_numVPLs);
        return "VPLIntegrator []";
    }

private:
    void generateIndirectVPLs(const Scene *scene, std::unique_ptr<Sampler> &sampler,
                              const VPL &vpl, std::vector<VPL> &vpls)
    {
        Ray3f ray(vpl.position, Warp::squareToUniformHemisphere(sampler->next2D()));
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
        }
    }

private:
    std::vector<VPL> m_vpls;
    int m_numVPLs;
};

NORI_REGISTER_CLASS(VPLIntegrator, "vpl");
NORI_NAMESPACE_END