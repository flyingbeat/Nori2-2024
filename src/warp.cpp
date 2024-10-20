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

Point2f Warp::squareToUniformDisk(const Point2f &sample)
{
    throw NoriException("Warp::squareToUniformDisk() is not yet implemented!");
}

float Warp::squareToUniformDiskPdf(const Point2f &p)
{
    throw NoriException("Warp::squareToUniformDiskPdf() is not yet implemented!");
}

Point2f Warp::squareToUniformTriangle(const Point2f &sample)
{
    float x = sample.x();
    float y = sample.y();
    if (x + y <= 1.0f)
        return sample;
    return Point2f(1.0f - x, 1.0f - y);
}

float Warp::squareToUniformTrianglePdf(const Point2f &p)
{
    return (p.x() >= 0 && p.y() >= 0 && p.x() + p.y() <= 1) ? 2.0f : 0.0f;
}

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

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha)
{
    throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha)
{
    throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END
