/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample)
{
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample)
{
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample)
{
    throw NoriException("Warp::squareToTent() is not yet implemented!");
}

float Warp::squareToTentPdf(const Point2f &p)
{
    throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
}

// REFERENCES: https://pbr-book.org/4ed/Shapes/Disks#fragment-DiskPublicMethods-3
Point2f Warp::squareToUniformDisk(const Point2f &sample)
{
    float u = sample.x();
    float v = sample.y();
    float r = sqrt(u);
    float theta = 2 * M_PI * v;

    float x = r * cos(theta);
    float y = r * sin(theta);

    return Point2f(x, y);
}

// REFERENCES: https://pbr-book.org/4ed/Shapes/Disks#fragment-DiskPublicMethods-3
float Warp::squareToUniformDiskPdf(const Point2f &p)
{
    float r = p.norm();
    // area of unit disk = PI * 1^2 = PI
    return r <= 1 ? 1 / M_PI : 0;
}

// REFERENCES: https://pbr-book.org/4ed/Shapes/Triangle_Meshes
Point2f Warp::squareToUniformTriangle(const Point2f &sample)
{
    float x = sample.x();
    float y = sample.y();
    if (x + y <= 1.0f)
        return sample;
    return Point2f(1.0f - x, 1.0f - y);
}

// REFERENCES: https://pbr-book.org/4ed/Shapes/Triangle_Meshes
float Warp::squareToUniformTrianglePdf(const Point2f &p)
{
    return (p.x() >= 0 && p.y() >= 0 && p.x() + p.y() <= 1) ? 2.0f : 0.0f;
}

// REFERENCES: https://pbr-book.org/4ed/Shapes/Spheres
Vector3f Warp::squareToUniformSphere(const Point2f &sample)
{
    float u = sample.x();
    float v = sample.y();

    float phi = 2.0f * M_PI * u;
    float theta = acos(1.0f - 2.0f * v);

    float x = sin(theta) * cos(phi);
    float y = sin(theta) * sin(phi);
    float z = cos(theta);

    return Vector3f(x, y, z);
}

// REFERENCES: https://pbr-book.org/4ed/Shapes/Spheres
float Warp::squareToUniformSpherePdf(const Vector3f &v)
{
    return 1.0f / (4.0f * M_PI);
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample)
{
    float u = sample.x();
    float v = sample.y();

    float phi = 2.0f * M_PI * u;
    float theta = acos(1.0f - v);

    float x = sin(theta) * cos(phi);
    float y = sin(theta) * sin(phi);
    float z = cos(theta);

    return Vector3f(x, y, z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v)
{
    return v.z() >= 0 ? 1.0f / (2.0f * M_PI) : 0.0f;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample)
{
    float u = sample.x();
    float v = sample.y();

    float phi = 2.0f * M_PI * v;
    float theta = std::sqrt(u);

    float x = theta * std::cos(phi);
    float y = theta * std::sin(phi);
    float z = std::sqrt(1.0f - u);

    return Vector3f(x, y, z);
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v)
{
    return (v.z() < 0.0f || v.z() > 1.0f) ? 0.0f : v.z() / M_PI;
}

// REFERENCES: https://www.graphics.cornell.edu/~bjw/microfacetbsdf.pdf (28)
Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha)
{
    float u = sample.x();
    float v = sample.y();

    float phi = 2.0f * M_PI * v;
    float theta = atanf(sqrtf(-(pow(alpha, 2) * log(u))));
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

// REFERENCES:
// https://www.pbr-book.org/3ed-2018/Reflection_Models/Microfacet_Models
// https://www.pbr-book.org/3ed-2018/Reflection_Models/Microfacet_Models#eq:beckmann-d
float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha)
{
    float cosTheta = Frame::cosTheta(m);
    float sin2Theta = std::max(0.0f, 1.0f - powf(cosTheta, 2));
    float tan2Theta = sin2Theta / powf(cosTheta, 2);
    if (cosTheta <= 0 || std::isinf(tan2Theta))
        return 0.;

    float alpha2 = powf(alpha, 2);
    float cos4Theta = powf(cosTheta, 4);

    float D = expf(-tan2Theta * (powf(Frame::cosPhi(m), 2) / alpha2 +
                                 powf(Frame::sinPhi(m), 2) / alpha2)) /
              (M_PI * alpha2 * cos4Theta);
    return D * cosTheta;
}

NORI_NAMESPACE_END
