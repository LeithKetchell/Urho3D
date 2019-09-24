//
// Copyright (c) 2008-2019 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include "../Precompiled.h"

#include "../Math/BoundingBox.h"
#include "../Math/Frustum.h"
#include "../Math/Ray.h"

#include "../DebugNew.h"

namespace Urho3D
{

Vector3 Ray::ClosestPoint(const Ray& ray) const
{
    // Algorithm based on http://paulbourke.net/geometry/lineline3d/
    Vector3 p13 = origin_ - ray.origin_;
    Vector3 p43 = ray.direction_;
    Vector3 p21 = direction_;

    float d1343 = p13.DotProduct(p43);
    float d4321 = p43.DotProduct(p21);
    float d1321 = p13.DotProduct(p21);
    float d4343 = p43.DotProduct(p43);
    float d2121 = p21.DotProduct(p21);

    float d = d2121 * d4343 - d4321 * d4321;
    if (Abs(d) < M_EPSILON)
        return origin_;
    float n = d1343 * d4321 - d1321 * d4343;
    float a = n / d;

    return origin_ + a * direction_;
}

float Ray::HitDistance(const Plane& plane) const
{
    float d = plane.normal_.DotProduct(direction_);
    if (Abs(d) >= M_EPSILON)
    {
        float t = -(plane.normal_.DotProduct(origin_) + plane.d_) / d;
        if (t >= 0.0f)
            return t;
        else
            return M_INFINITY;
    }
    else
        return M_INFINITY;
}

float Ray::HitDistance(const BoundingBox& box) const
{
    // If undefined, no hit (infinite distance)
    if (!box.Defined())
        return M_INFINITY;

    // Check for ray origin being inside the box
    if (box.IsInside(origin_))
        return 0.0f;

    float dist = M_INFINITY;

    // Check for intersecting in the X-direction
    if (origin_.x_ < box.min_.x_ && direction_.x_ > 0.0f)
    {
        float x = (box.min_.x_ - origin_.x_) / direction_.x_;
        if (x < dist)
        {
            Vector3 point = origin_ + x * direction_;
            if (point.y_ >= box.min_.y_ && point.y_ <= box.max_.y_ && point.z_ >= box.min_.z_ && point.z_ <= box.max_.z_)
                dist = x;
        }
    }
    if (origin_.x_ > box.max_.x_ && direction_.x_ < 0.0f)
    {
        float x = (box.max_.x_ - origin_.x_) / direction_.x_;
        if (x < dist)
        {
            Vector3 point = origin_ + x * direction_;
            if (point.y_ >= box.min_.y_ && point.y_ <= box.max_.y_ && point.z_ >= box.min_.z_ && point.z_ <= box.max_.z_)
                dist = x;
        }
    }
    // Check for intersecting in the Y-direction
    if (origin_.y_ < box.min_.y_ && direction_.y_ > 0.0f)
    {
        float x = (box.min_.y_ - origin_.y_) / direction_.y_;
        if (x < dist)
        {
            Vector3 point = origin_ + x * direction_;
            if (point.x_ >= box.min_.x_ && point.x_ <= box.max_.x_ && point.z_ >= box.min_.z_ && point.z_ <= box.max_.z_)
                dist = x;
        }
    }
    if (origin_.y_ > box.max_.y_ && direction_.y_ < 0.0f)
    {
        float x = (box.max_.y_ - origin_.y_) / direction_.y_;
        if (x < dist)
        {
            Vector3 point = origin_ + x * direction_;
            if (point.x_ >= box.min_.x_ && point.x_ <= box.max_.x_ && point.z_ >= box.min_.z_ && point.z_ <= box.max_.z_)
                dist = x;
        }
    }
    // Check for intersecting in the Z-direction
    if (origin_.z_ < box.min_.z_ && direction_.z_ > 0.0f)
    {
        float x = (box.min_.z_ - origin_.z_) / direction_.z_;
        if (x < dist)
        {
            Vector3 point = origin_ + x * direction_;
            if (point.x_ >= box.min_.x_ && point.x_ <= box.max_.x_ && point.y_ >= box.min_.y_ && point.y_ <= box.max_.y_)
                dist = x;
        }
    }
    if (origin_.z_ > box.max_.z_ && direction_.z_ < 0.0f)
    {
        float x = (box.max_.z_ - origin_.z_) / direction_.z_;
        if (x < dist)
        {
            Vector3 point = origin_ + x * direction_;
            if (point.x_ >= box.min_.x_ && point.x_ <= box.max_.x_ && point.y_ >= box.min_.y_ && point.y_ <= box.max_.y_)
                dist = x;
        }
    }

    return dist;
}

float Ray::HitDistance(const Frustum& frustum, bool solidInside) const
{
    float maxOutside = 0.0f;
    float minInside = M_INFINITY;
    bool allInside = true;

    for (const auto& plane : frustum.planes_)
    {
        float distance = HitDistance(plane);

        if (plane.Distance(origin_) < 0.0f)
        {
            maxOutside = Max(maxOutside, distance);
            allInside = false;
        }
        else
            minInside = Min(minInside, distance);
    }

    if (allInside)
        return solidInside ? 0.0f : minInside;
    else if (maxOutside <= minInside)
        return maxOutside;
    else
        return M_INFINITY;
}

float Ray::HitDistance(const Sphere& sphere) const
{
    Vector3 centeredOrigin = origin_ - sphere.center_;
    float squaredRadius = sphere.radius_ * sphere.radius_;

    // Check if ray originates inside the sphere
    if (centeredOrigin.LengthSquared() <= squaredRadius)
        return 0.0f;

    // Calculate intersection by quadratic equation
    float a = direction_.DotProduct(direction_);
    float b = 2.0f * centeredOrigin.DotProduct(direction_);
    float c = centeredOrigin.DotProduct(centeredOrigin) - squaredRadius;
    float d = b * b - 4.0f * a * c;

    // No solution
    if (d < 0.0f)
        return M_INFINITY;

    // Get the nearer solution
    float dSqrt = sqrtf(d);
    float dist = (-b - dSqrt) / (2.0f * a);
    if (dist >= 0.0f)
        return dist;
    else
        return (-b + dSqrt) / (2.0f * a);
}


    // Calculate the normal in a point on the surface
// it is a vertical vector in the bases and a vector
// having the direction of the vector from the axis to the point
Vector3 GetCylinderNormal (const Vector3& p, float radius, float height)
{
	// Point is on one of the bases
	if (p.x_<radius && p.x_>-radius && p.z_<radius && p.z_>-radius)
	{
		double epsilon = 0.00000001;
		if (p.y_ < height+epsilon && p.y_>height-epsilon){
			return Vector3::UP;
		}
		if (p.y_ < epsilon && p.y_>-epsilon){
			return Vector3::DOWN;
		}
	}

	// Point is on lateral surface
 	Vector3 c0 (0, p.y_, 0);
 	Vector3 v = (p-c0).Normalized();
 	return v;
}

float Ray::intersectOBB(const BoundingBox& boxbounds, const Matrix3x4& boxXform, Vector3& hitpoint){

    /// Transform ray to local space of box
    Matrix3x4 inv = boxXform.Inverse();
    Ray ray=Transformed(inv);

    /// Compute ray/box intersection in box local space
    float d = ray.HitDistance(boxbounds);

    /// If we hit the box, compute localspace hitpoint and transform it back to worldspace
    if(d!=M_INFINITY)
        hitpoint = boxXform * (ray.origin_ + ray.direction_ * d);

    return d;

}

// Calculate intersection with cylinder base having center c = either <0,0,0> or <0,height,0>
// We do this by calculating the intersection with the Y plane,
// and then checking if the intersection is within the 2D circle (XZ).
bool intersectCylinderBase (const Ray& ray, float radius, float height, const Vector3& c, double& dist, Vector3& hitPoint)
{
	Vector3 normal = GetCylinderNormal (c, radius, height);

	double D =  -normal.DotProduct(c);
	double phi = normal.DotProduct(ray.direction_);

	if (phi==0)
		return false;

	dist = - (normal.DotProduct(ray.origin_)+D) / phi;

	const double epsilon = 0.00000001;
	if (dist < epsilon)
		return false;

	Vector3 p  = ray.origin_+dist*ray.direction_;
	if (p.x_*p.x_+p.z_*p.z_-radius*radius > epsilon)
		return false;

	hitPoint=p;
	return true;
}


// Find intersection with infinite cylinder with center in the origin
// to do that, translate the ray origin so that the center of the bottom base
// is at the origin, then calculate intersection with the canonical infinite cylinder
// and check if the ray intersects the lateral surface of the cylinder within our
// bases, if not check if it's intersecting the bases and if not, it's not intersecting
// our actual cylinder
float Ray::intersectCylinder (const Vector3& center, const Vector3& normal, float radius, float height, Vector3& hitPoint)
{

    /// Compute a suitable rotation from a base vector to the desired direction
    Quaternion q;
    q.FromRotationTo(Vector3::UP, normal);

    /// Compute transform from cylinder local space to world space
    Matrix3x4 mat(center, q, Vector3::ONE);

    /// Compute transform from world space to cylinder local space
    Matrix3x4 inv=mat.Inverse();

    /// Transform ray from world space to cylinder local space
    Ray ray = Transformed(inv);

    /// We wish to compute a positive value for t
    float t;

	// Note the ray origin (transformed into cylinder's local space)
	Vector3 p0 = ray.origin_;

	// coefficients for the intersection equation
	// got them mathematically intersecting the line equation with the cylinder equation
	double a = ray.direction_.x_*ray.direction_.x_+ray.direction_.z_*ray.direction_.z_;
	double b = ray.direction_.x_*p0.x_ +ray.direction_.z_*p0.z_;
	double c = p0.x_*p0.x_+p0.z_*p0.z_-radius*radius;

	double delta = b*b - a*c;

	//use epsilon because of computation errors between doubles
	const double epsilon = 0.00000001;

	// delta < 0 means no intersections
	if (delta < epsilon)
		return M_INFINITY;

	// nearest intersection
	t = (-b - sqrt (delta))/a;

	// t<0 means the intersection is behind the ray origin
	// which we don't want
	if (t<=epsilon)
		return M_INFINITY;

	hitPoint = p0 + t * ray.direction_;


	/// check if we intersect one of the cylinder caps (aka bases)
	if (hitPoint.y_ > height+epsilon || hitPoint.y_ < -epsilon) {
		double dist;
		Vector3 hp1, hp2;

        /// Check for intersection with upper cap
		bool b1 = intersectCylinderBase(ray, radius, height, Vector3(0, height, 0), dist, hp1);
		if(b1) { t=dist; hitPoint=hp1; }

		/// Check for intersection with lower cap
		bool b2 = intersectCylinderBase (ray, radius, height, Vector3::ZERO, dist, hp2);
		if(b2 && dist>epsilon && t>=dist) { t=dist; hitPoint=hp2; }

        /// If there's NO intersection...
		if(! (b1||b2) )
            t=M_INFINITY;
	}

    /// If t is valid, then transform the hitpoint from cylinder space to worldspace
    if(t!=M_INFINITY)
        hitPoint = mat * hitPoint;

	return t;

}

float Ray::HitDistance(const Vector3& v0, const Vector3& v1, const Vector3& v2, Vector3* outNormal, Vector3* outBary) const
{
    // Based on Fast, Minimum Storage Ray/Triangle Intersection by Möller & Trumbore
    // http://www.graphics.cornell.edu/pubs/1997/MT97.pdf
    // Calculate edge vectors
    Vector3 edge1(v1 - v0);
    Vector3 edge2(v2 - v0);

    // Calculate determinant & check backfacing
    Vector3 p(direction_.CrossProduct(edge2));
    float det = edge1.DotProduct(p);
    if (det >= M_EPSILON)
    {
        // Calculate u & v parameters and test
        Vector3 t(origin_ - v0);
        float u = t.DotProduct(p);
        if (u >= 0.0f && u <= det)
        {
            Vector3 q(t.CrossProduct(edge1));
            float v = direction_.DotProduct(q);
            if (v >= 0.0f && u + v <= det)
            {
                float distance = edge2.DotProduct(q) / det;
                // Discard hits behind the ray
                if (distance >= 0.0f)
                {
                    // There is an intersection, so calculate distance & optional normal
                    if (outNormal)
                        *outNormal = edge1.CrossProduct(edge2);
                    if (outBary)
                        *outBary = Vector3(1 - (u / det) - (v / det), u / det, v / det);

                    return distance;
                }
            }
        }
    }

    return M_INFINITY;
}

float Ray::HitDistance(const void* vertexData, unsigned vertexStride, unsigned vertexStart, unsigned vertexCount,
    Vector3* outNormal, Vector2* outUV, unsigned uvOffset) const
{
    float nearest = M_INFINITY;
    const unsigned char* vertices = ((const unsigned char*)vertexData) + vertexStart * vertexStride;
    unsigned index = 0, nearestIdx = M_MAX_UNSIGNED;
    Vector3 barycentric;
    Vector3* outBary = outUV ? &barycentric : nullptr;

    while (index + 2 < vertexCount)
    {
        const Vector3& v0 = *((const Vector3*)(&vertices[index * vertexStride]));
        const Vector3& v1 = *((const Vector3*)(&vertices[(index + 1) * vertexStride]));
        const Vector3& v2 = *((const Vector3*)(&vertices[(index + 2) * vertexStride]));
        float distance = HitDistance(v0, v1, v2, outNormal, outBary);
        if (distance < nearest)
        {
            nearestIdx = index;
            nearest = distance;
        }
        index += 3;
    }

    if (outUV)
    {
        if (nearestIdx == M_MAX_UNSIGNED)
            *outUV = Vector2::ZERO;
        else
        {
            // Interpolate the UV coordinate using barycentric coordinate
            const Vector2& uv0 = *((const Vector2*)(&vertices[uvOffset + nearestIdx * vertexStride]));
            const Vector2& uv1 = *((const Vector2*)(&vertices[uvOffset + (nearestIdx + 1) * vertexStride]));
            const Vector2& uv2 = *((const Vector2*)(&vertices[uvOffset + (nearestIdx + 2) * vertexStride]));
            *outUV = Vector2(uv0.x_ * barycentric.x_ + uv1.x_ * barycentric.y_ + uv2.x_ * barycentric.z_,
                uv0.y_ * barycentric.x_ + uv1.y_ * barycentric.y_ + uv2.y_ * barycentric.z_);
        }
    }

    return nearest;
}

float Ray::HitDistance(const void* vertexData, unsigned vertexStride, const void* indexData, unsigned indexSize,
    unsigned indexStart, unsigned indexCount, Vector3* outNormal, Vector2* outUV, unsigned uvOffset) const
{
    float nearest = M_INFINITY;
    const auto* vertices = (const unsigned char*)vertexData;
    Vector3 barycentric;
    Vector3* outBary = outUV ? &barycentric : nullptr;

    // 16-bit indices
    if (indexSize == sizeof(unsigned short))
    {
        const unsigned short* indices = ((const unsigned short*)indexData) + indexStart;
        const unsigned short* indicesEnd = indices + indexCount;
        const unsigned short* nearestIndices = nullptr;

        while (indices < indicesEnd)
        {
            const Vector3& v0 = *((const Vector3*)(&vertices[indices[0] * vertexStride]));
            const Vector3& v1 = *((const Vector3*)(&vertices[indices[1] * vertexStride]));
            const Vector3& v2 = *((const Vector3*)(&vertices[indices[2] * vertexStride]));
            float distance = HitDistance(v0, v1, v2, outNormal, outBary);
            if (distance < nearest)
            {
                nearestIndices = indices;
                nearest = distance;
            }
            indices += 3;
        }

        if (outUV)
        {
            if (nearestIndices == nullptr)
                *outUV = Vector2::ZERO;
            else
            {
                // Interpolate the UV coordinate using barycentric coordinate
                const Vector2& uv0 = *((const Vector2*)(&vertices[uvOffset + nearestIndices[0] * vertexStride]));
                const Vector2& uv1 = *((const Vector2*)(&vertices[uvOffset + nearestIndices[1] * vertexStride]));
                const Vector2& uv2 = *((const Vector2*)(&vertices[uvOffset + nearestIndices[2] * vertexStride]));
                *outUV = Vector2(uv0.x_ * barycentric.x_ + uv1.x_ * barycentric.y_ + uv2.x_ * barycentric.z_,
                    uv0.y_ * barycentric.x_ + uv1.y_ * barycentric.y_ + uv2.y_ * barycentric.z_);
            }
        }
    }
    // 32-bit indices
    else
    {
        const unsigned* indices = ((const unsigned*)indexData) + indexStart;
        const unsigned* indicesEnd = indices + indexCount;
        const unsigned* nearestIndices = nullptr;

        while (indices < indicesEnd)
        {
            const Vector3& v0 = *((const Vector3*)(&vertices[indices[0] * vertexStride]));
            const Vector3& v1 = *((const Vector3*)(&vertices[indices[1] * vertexStride]));
            const Vector3& v2 = *((const Vector3*)(&vertices[indices[2] * vertexStride]));
            float distance = HitDistance(v0, v1, v2, outNormal, outBary);
            if (distance < nearest)
            {
                nearestIndices = indices;
                nearest = distance;
            }
            indices += 3;
        }

        if (outUV)
        {
            if (nearestIndices == nullptr)
                *outUV = Vector2::ZERO;
            else
            {
                // Interpolate the UV coordinate using barycentric coordinate
                const Vector2& uv0 = *((const Vector2*)(&vertices[uvOffset + nearestIndices[0] * vertexStride]));
                const Vector2& uv1 = *((const Vector2*)(&vertices[uvOffset + nearestIndices[1] * vertexStride]));
                const Vector2& uv2 = *((const Vector2*)(&vertices[uvOffset + nearestIndices[2] * vertexStride]));
                *outUV = Vector2(uv0.x_ * barycentric.x_ + uv1.x_ * barycentric.y_ + uv2.x_ * barycentric.z_,
                    uv0.y_ * barycentric.x_ + uv1.y_ * barycentric.y_ + uv2.y_ * barycentric.z_);
            }
        }
    }

    return nearest;
}

bool Ray::InsideGeometry(const void* vertexData, unsigned vertexSize, unsigned vertexStart, unsigned vertexCount) const
{
    float currentFrontFace = M_INFINITY;
    float currentBackFace = M_INFINITY;
    const unsigned char* vertices = ((const unsigned char*)vertexData) + vertexStart * vertexSize;
    unsigned index = 0;

    while (index + 2 < vertexCount)
    {
        const Vector3& v0 = *((const Vector3*)(&vertices[index * vertexSize]));
        const Vector3& v1 = *((const Vector3*)(&vertices[(index + 1) * vertexSize]));
        const Vector3& v2 = *((const Vector3*)(&vertices[(index + 2) * vertexSize]));
        float frontFaceDistance = HitDistance(v0, v1, v2);
        float backFaceDistance = HitDistance(v2, v1, v0);
        currentFrontFace = Min(frontFaceDistance > 0.0f ? frontFaceDistance : M_INFINITY, currentFrontFace);
        // A backwards face is just a regular one, with the vertices in the opposite order. This essentially checks backfaces by
        // checking reversed frontfaces
        currentBackFace = Min(backFaceDistance > 0.0f ? backFaceDistance : M_INFINITY, currentBackFace);
        index += 3;
    }

    // If the closest face is a backface, that means that the ray originates from the inside of the geometry
    // NOTE: there may be cases where both are equal, as in, no collision to either. This is prevented in the most likely case
    // (ray doesn't hit either) by this conditional
    if (currentFrontFace != M_INFINITY || currentBackFace != M_INFINITY)
        return currentBackFace < currentFrontFace;

    // It is still possible for two triangles to be equally distant from the triangle, however, this is extremely unlikely.
    // As such, it is safe to assume they are not
    return false;
}

bool Ray::InsideGeometry(const void* vertexData, unsigned vertexSize, const void* indexData, unsigned indexSize,
    unsigned indexStart, unsigned indexCount) const
{
    float currentFrontFace = M_INFINITY;
    float currentBackFace = M_INFINITY;
    const auto* vertices = (const unsigned char*)vertexData;

    // 16-bit indices
    if (indexSize == sizeof(unsigned short))
    {
        const unsigned short* indices = ((const unsigned short*)indexData) + indexStart;
        const unsigned short* indicesEnd = indices + indexCount;

        while (indices < indicesEnd)
        {
            const Vector3& v0 = *((const Vector3*)(&vertices[indices[0] * vertexSize]));
            const Vector3& v1 = *((const Vector3*)(&vertices[indices[1] * vertexSize]));
            const Vector3& v2 = *((const Vector3*)(&vertices[indices[2] * vertexSize]));
            float frontFaceDistance = HitDistance(v0, v1, v2);
            float backFaceDistance = HitDistance(v2, v1, v0);
            currentFrontFace = Min(frontFaceDistance > 0.0f ? frontFaceDistance : M_INFINITY, currentFrontFace);
            // A backwards face is just a regular one, with the vertices in the opposite order. This essentially checks backfaces by
            // checking reversed frontfaces
            currentBackFace = Min(backFaceDistance > 0.0f ? backFaceDistance : M_INFINITY, currentBackFace);
            indices += 3;
        }
    }
    // 32-bit indices
    else
    {
        const unsigned* indices = ((const unsigned*)indexData) + indexStart;
        const unsigned* indicesEnd = indices + indexCount;

        while (indices < indicesEnd)
        {
            const Vector3& v0 = *((const Vector3*)(&vertices[indices[0] * vertexSize]));
            const Vector3& v1 = *((const Vector3*)(&vertices[indices[1] * vertexSize]));
            const Vector3& v2 = *((const Vector3*)(&vertices[indices[2] * vertexSize]));
            float frontFaceDistance = HitDistance(v0, v1, v2);
            float backFaceDistance = HitDistance(v2, v1, v0);
            currentFrontFace = Min(frontFaceDistance > 0.0f ? frontFaceDistance : M_INFINITY, currentFrontFace);
            // A backwards face is just a regular one, with the vertices in the opposite order. This essentially checks backfaces by
            // checking reversed frontfaces
            currentBackFace = Min(backFaceDistance > 0.0f ? backFaceDistance : M_INFINITY, currentBackFace);
            indices += 3;
        }
    }

    // If the closest face is a backface, that means that the ray originates from the inside of the geometry
    // NOTE: there may be cases where both are equal, as in, no collision to either. This is prevented in the most likely case
    // (ray doesn't hit either) by this conditional
    if (currentFrontFace != M_INFINITY || currentBackFace != M_INFINITY)
        return currentBackFace < currentFrontFace;

    // It is still possible for two triangles to be equally distant from the triangle, however, this is extremely unlikely.
    // As such, it is safe to assume they are not
    return false;
}

Ray Ray::Transformed(const Matrix3x4& transform) const
{
    Ray ret;
    ret.origin_ = transform * origin_;
    ret.direction_ = transform * Vector4(direction_, 0.0f);
    return ret;
}

}
