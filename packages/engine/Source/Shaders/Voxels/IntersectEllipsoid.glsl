// See IntersectionUtils.glsl for the definitions of Ray, Intersections,
// setIntersection, setIntersectionPair, INF_HIT, NO_HIT

/* Ellipsoid defines (set in Scene/VoxelEllipsoidShape.js)
#define ELLIPSOID_HAS_RENDER_BOUNDS_LONGITUDE
#define ELLIPSOID_HAS_RENDER_BOUNDS_LONGITUDE_RANGE_EQUAL_ZERO
#define ELLIPSOID_HAS_RENDER_BOUNDS_LONGITUDE_RANGE_UNDER_HALF
#define ELLIPSOID_HAS_RENDER_BOUNDS_LONGITUDE_RANGE_EQUAL_HALF
#define ELLIPSOID_HAS_RENDER_BOUNDS_LONGITUDE_RANGE_OVER_HALF
#define ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE
#define ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MAX_UNDER_HALF
#define ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MAX_EQUAL_HALF
#define ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MAX_OVER_HALF
#define ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MIN_UNDER_HALF
#define ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MIN_EQUAL_HALF
#define ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MIN_OVER_HALF
#define ELLIPSOID_HAS_RENDER_BOUNDS_HEIGHT_MAX
#define ELLIPSOID_HAS_RENDER_BOUNDS_HEIGHT_MIN
#define ELLIPSOID_HAS_RENDER_BOUNDS_HEIGHT_FLAT
#define ELLIPSOID_INTERSECTION_INDEX_LONGITUDE
#define ELLIPSOID_INTERSECTION_INDEX_LATITUDE_MAX
#define ELLIPSOID_INTERSECTION_INDEX_LATITUDE_MIN
#define ELLIPSOID_INTERSECTION_INDEX_HEIGHT_MAX
#define ELLIPSOID_INTERSECTION_INDEX_HEIGHT_MIN
*/

#if defined(ELLIPSOID_HAS_RENDER_BOUNDS_LONGITUDE)
    uniform vec2 u_ellipsoidRenderLongitudeMinMax;
#endif
#if defined(ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MIN_UNDER_HALF) || defined(ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MIN_OVER_HALF) || defined(ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MAX_UNDER_HALF) || defined(ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MAX_OVER_HALF)
    uniform vec2 u_ellipsoidRenderLatitudeCosSqrHalfMinMax;
#endif
#if defined(ELLIPSOID_HAS_RENDER_BOUNDS_HEIGHT_MAX)
    uniform float u_ellipsoidInverseOuterScaleUv;
#endif
#if defined(ELLIPSOID_HAS_RENDER_BOUNDS_HEIGHT_MIN)
    uniform float u_ellipsoidInverseInnerScaleUv;
#endif

RayShapeIntersection intersectZPlane(Ray ray)
{
    float o = ray.pos.z;
    float d = ray.dir.z;
    float t = -o / d;

    bool entry = (t >= 0.0) != (o > 0.0);
    float z = entry ? -1.0 : 1.0;
    vec4 intersect = vec4(0.0, 0.0, z, t);
    vec4 farSide = vec4(normalize(ray.dir), INF_HIT);

    if (entry) {
        return RayShapeIntersection(intersect, farSide);
    } else {
        return RayShapeIntersection(-1.0 * farSide, intersect);
    }
}

vec4 intersectHalfPlane(Ray ray, float angle) {
    vec2 o = ray.pos.xy;
    vec2 d = ray.dir.xy;
    vec2 planeDirection = vec2(cos(angle), sin(angle));
    vec2 planeNormal = vec2(planeDirection.y, -planeDirection.x);

    float a = dot(o, planeNormal);
    float b = dot(d, planeNormal);
    float t = -a / b;

    vec2 p = o + t * d;
    bool outside = dot(p, planeDirection) < 0.0;
    if (outside) return vec4(-INF_HIT, +INF_HIT, NO_HIT, NO_HIT);

    return vec4(-INF_HIT, t, t, +INF_HIT);
}

vec2 intersectHalfSpace(Ray ray, float angle)
{
    vec2 o = ray.pos.xy;
    vec2 d = ray.dir.xy;
    vec2 n = vec2(sin(angle), -cos(angle));

    float a = dot(o, n);
    float b = dot(d, n);
    float t = -a / b;
    float s = sign(a);

    if (t >= 0.0 != s >= 0.0) return vec2(t, +INF_HIT);
    else return vec2(-INF_HIT, t);
}

vec2 intersectRegularWedge(Ray ray, float minAngle, float maxAngle)
{
    vec2 o = ray.pos.xy;
    vec2 d = ray.dir.xy;
    vec2 n1 = vec2(sin(minAngle), -cos(minAngle));
    vec2 n2 = vec2(-sin(maxAngle), cos(maxAngle));

    float a1 = dot(o, n1);
    float a2 = dot(o, n2);
    float b1 = dot(d, n1);
    float b2 = dot(d, n2);

    float t1 = -a1 / b1;
    float t2 = -a2 / b2;
    float s1 = sign(a1);
    float s2 = sign(a2);

    float tmin = min(t1, t2);
    float tmax = max(t1, t2);
    float smin = tmin == t1 ? s1 : s2;
    float smax = tmin == t1 ? s2 : s1;

    bool e = tmin >= 0.0;
    bool f = tmax >= 0.0;
    bool g = smin >= 0.0;
    bool h = smax >= 0.0;

    if (e != g && f == h) return vec2(tmin, tmax);
    else if (e == g && f == h) return vec2(-INF_HIT, tmin);
    else if (e != g && f != h) return vec2(tmax, +INF_HIT);
    else return vec2(NO_HIT);
}

vec4 intersectFlippedWedge(Ray ray, float minAngle, float maxAngle)
{
    vec2 planeIntersectMin = intersectHalfSpace(ray, minAngle);
    vec2 planeIntersectMax = intersectHalfSpace(ray, maxAngle + czm_pi);
    return vec4(planeIntersectMin, planeIntersectMax);
}

RayShapeIntersection intersectUnitSphere(Ray ray, bool convex)
{
    vec3 position = ray.pos;
    vec3 direction = ray.dir;

    float a = dot(direction, direction);
    float b = dot(direction, position);
    float c = dot(position, position) - 1.0;
    float determinant = b * b - a * c;

    if (determinant < 0.0) {
        vec4 miss = vec4(normalize(direction), NO_HIT);
        return RayShapeIntersection(miss, miss);
    }

    determinant = sqrt(determinant);
    float t1 = (-b - determinant) / a;
    float t2 = (-b + determinant) / a;

    float tmin = min(t1, t2);
    float tmax = max(t1, t2);

    float directionScale = convex ? 1.0 : -1.0;
    vec3 dmin = directionScale * normalize(position + tmin * direction);
    vec3 dmax = directionScale * normalize(position + tmax * direction);

    return RayShapeIntersection(vec4(dmin, tmin), vec4(dmax, tmax));
}

/**
 * Given a circular cone around the z-axis, with apex at the origin,
 * find the parametric distance(s) along a ray where that ray intersects
 * the cone.
 * The cone opening angle is described by the squared cosine of
 * its half-angle (the angle between the Z-axis and the surface)
 */
vec2 intersectDoubleEndedCone(Ray ray, float cosSqrHalfAngle)
{
    vec3 o = ray.pos;
    vec3 d = ray.dir;
    float sinSqrHalfAngle = 1.0 - cosSqrHalfAngle;
    // a = d.z * d.z - dot(d, d) * cosSqrHalfAngle;
    float aSin = d.z * d.z * sinSqrHalfAngle;
    float aCos = -dot(d.xy, d.xy) * cosSqrHalfAngle;
    float a = aSin + aCos;
    // b = d.z * o.z - dot(o, d) * cosSqrHalfAngle;
    float bSin = d.z * o.z * sinSqrHalfAngle;
    float bCos = -dot(o.xy, d.xy) * cosSqrHalfAngle;
    float b = bSin + bCos;
    // c = o.z * o.z - dot(o, o) * cosSqrHalfAngle;
    float cSin = o.z * o.z * sinSqrHalfAngle;
    float cCos = -dot(o.xy, o.xy) * cosSqrHalfAngle;
    float c = cSin + cCos;
    // determinant = b * b - a * c. But bSin * bSin = aSin * cSin.
    // Avoid subtractive cancellation by expanding to eliminate these terms
    float det = 2.0 * bSin * bCos + bCos * bCos - aSin * cCos - aCos * cSin - aCos * cCos;

    if (det < 0.0) {
        return vec2(NO_HIT);
    } else if (a == 0.0) {
        // Ray is parallel to cone surface
        return (b == 0.0)
            ? vec2(NO_HIT) // Ray is on cone surface
            : vec2(-0.5 * c / b, NO_HIT);
    }

    det = sqrt(det);
    float t1 = (-b - det) / a;
    float t2 = (-b + det) / a;
    float tmin = min(t1, t2);
    float tmax = max(t1, t2);
    return vec2(tmin, tmax);
}

vec4 intersectFlippedCone(Ray ray, float cosSqrHalfAngle) {
    vec2 intersect = intersectDoubleEndedCone(ray, cosSqrHalfAngle);

    if (intersect.x == NO_HIT) {
        return vec4(-INF_HIT, +INF_HIT, NO_HIT, NO_HIT);
    }

    vec3 o = ray.pos;
    vec3 d = ray.dir;
    float tmin = intersect.x;
    float tmax = intersect.y;
    float zmin = o.z + tmin * d.z;
    float zmax = o.z + tmax * d.z;

    // One interval
    if (zmin < 0.0 && zmax < 0.0) return vec4(-INF_HIT, +INF_HIT, NO_HIT, NO_HIT);
    else if (zmin < 0.0) return vec4(-INF_HIT, tmax, NO_HIT, NO_HIT);
    else if (zmax < 0.0) return vec4(tmin, +INF_HIT, NO_HIT, NO_HIT);
    // Two intervals
    else return vec4(-INF_HIT, tmin, tmax, +INF_HIT);
}

RayShapeIntersection intersectRegularCone(Ray ray, float cosSqrHalfAngle) {
    vec2 intersect = intersectDoubleEndedCone(ray, cosSqrHalfAngle);

    vec4 miss = vec4(normalize(ray.dir), NO_HIT);
    vec4 farSide = vec4(normalize(ray.dir), INF_HIT);

    if (intersect.x == NO_HIT) {
        return RayShapeIntersection(miss, miss);
    }

    // Find the points of intersection
    float tmin = intersect.x;
    float tmax = intersect.y;
    vec3 p0 = ray.pos + tmin * ray.dir;
    vec3 p1 = ray.pos + tmax * ray.dir;

    // Find the surface normals at the intersection points (directed inside the cone)
    vec3 n0 = vec3(-p0.z * normalize(p0.xy), length(p0.xy));
    vec3 n1 = vec3(-p1.z * normalize(p1.xy), length(p1.xy));

    vec4 intersect0 = vec4(normalize(n0), tmin);
    vec4 intersect1 = vec4(normalize(n1), tmax);

    // Discard intersections with the shadow cone (below z == 0)
    if (p0.z < 0.0 && p1.z < 0.0) {
        return RayShapeIntersection(miss, miss);
    } else if (p0.z < 0.0) {
        return RayShapeIntersection(intersect1, farSide);
    } else if (p1.z < 0.0) {
        return RayShapeIntersection(-1.0 * farSide, intersect0);
    } else {
        return RayShapeIntersection(intersect0, intersect1);
    }
}

void intersectShape(in Ray ray, inout Intersections ix) {
    // Position is converted from [0,1] to [-1,+1] because shape intersections assume unit space is [-1,+1].
    // Direction is scaled as well to be in sync with position.
    ray.pos = ray.pos * 2.0 - 1.0;
    ray.dir *= 2.0;

    #if defined(ELLIPSOID_HAS_RENDER_BOUNDS_HEIGHT_MAX)
        Ray outerRay = Ray(ray.pos * u_ellipsoidInverseOuterScaleUv, ray.dir * u_ellipsoidInverseOuterScaleUv);
    #else
        Ray outerRay = ray;
    #endif

    // Outer ellipsoid
    RayShapeIntersection outerIntersect = intersectUnitSphere(outerRay, true);
    setShapeIntersection(ix, ELLIPSOID_INTERSECTION_INDEX_HEIGHT_MAX, outerIntersect);

    // Exit early if the outer ellipsoid was missed.
    if (outerIntersect.entry.w == NO_HIT) {
        return;
    }

    // Inner ellipsoid
    #if defined(ELLIPSOID_HAS_RENDER_BOUNDS_HEIGHT_FLAT)
        // When the ellipsoid is perfectly thin it's necessary to sandwich the
        // inner ellipsoid intersection inside the outer ellipsoid intersection.

        // Without this special case,
        // [outerMin, outerMax, innerMin, innerMax] will bubble sort to
        // [outerMin, innerMin, outerMax, innerMax] which will cause the back
        // side of the ellipsoid to be invisible because it will think the ray
        // is still inside the inner (negative) ellipsoid after exiting the
        // outer (positive) ellipsoid.

        // With this special case,
        // [outerMin, innerMin, innerMax, outerMax] will bubble sort to
        // [outerMin, innerMin, innerMax, outerMax] which will work correctly.

        // Note: If initializeIntersections() changes its sorting function
        // from bubble sort to something else, this code may need to change.
        vec4 innerEntry = vec4(-1.0 * outerIntersect.entry.xyz, outerIntersect.entry.w);
        vec4 innerExit = vec4(-1.0 * outerIntersect.exit.xyz, outerIntersect.exit.w);
        setSurfaceIntersection(ix, 0, outerIntersect.entry, true, true);   // positive, enter
        setSurfaceIntersection(ix, 1, innerEntry, false, true);  // negative, enter
        setSurfaceIntersection(ix, 2, innerExit, false, false); // negative, exit
        setSurfaceIntersection(ix, 3, outerIntersect.exit, true, false);  // positive, exit
    #elif defined(ELLIPSOID_HAS_RENDER_BOUNDS_HEIGHT_MIN)
        Ray innerRay = Ray(ray.pos * u_ellipsoidInverseInnerScaleUv, ray.dir * u_ellipsoidInverseInnerScaleUv);
        RayShapeIntersection innerIntersect = intersectUnitSphere(innerRay, false);

        if (innerIntersect.entry.w == NO_HIT) {
            setShapeIntersection(ix, ELLIPSOID_INTERSECTION_INDEX_HEIGHT_MIN, innerIntersect);
        } else {
            // When the ellipsoid is very large and thin it's possible for floating
            // point math to cause the ray to intersect the inner ellipsoid before
            // the outer ellipsoid. To prevent this from happening, clamp innerIntersect
            // to outerIntersect and sandwich the intersections like described above.
            //
            // In theory a similar fix is needed for cylinders, however it's more
            // complicated to implement because the inner shape is allowed to be
            // intersected first.
            innerIntersect.entry.w = max(innerIntersect.entry.w, outerIntersect.entry.w);
            innerIntersect.exit.w = min(innerIntersect.exit.w, outerIntersect.exit.w);
            setSurfaceIntersection(ix, 0, outerIntersect.entry, true, true);   // positive, enter
            setSurfaceIntersection(ix, 1, innerIntersect.entry, false, true);  // negative, enter
            setSurfaceIntersection(ix, 2, innerIntersect.exit, false, false); // negative, exit
            setSurfaceIntersection(ix, 3, outerIntersect.exit, true, false);  // positive, exit
        }
    #endif

    // Flip the ray because the intersection function expects a cone growing towards +Z.
    #if defined(ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MIN_UNDER_HALF) || defined(ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MIN_EQUAL_HALF) || defined(ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MAX_UNDER_HALF)
        Ray flippedRay = outerRay;
        flippedRay.dir.z *= -1.0;
        flippedRay.pos.z *= -1.0;
    #endif

    // Bottom cone
    #if defined(ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MIN_UNDER_HALF)
        RayShapeIntersection bottomConeIntersection = intersectRegularCone(flippedRay, u_ellipsoidRenderLatitudeCosSqrHalfMinMax.x);
        setShapeIntersection(ix, ELLIPSOID_INTERSECTION_INDEX_LATITUDE_MIN, bottomConeIntersection);
    #elif defined(ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MIN_EQUAL_HALF)
        RayShapeIntersection bottomConeIntersection = intersectZPlane(flippedRay);
        setShapeIntersection(ix, ELLIPSOID_INTERSECTION_INDEX_LATITUDE_MIN, bottomConeIntersection);
    #elif defined(ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MIN_OVER_HALF)
        vec4 bottomConeIntersection = intersectFlippedCone(ray, u_ellipsoidRenderLatitudeCosSqrHalfMinMax.x);
        setIntersectionPair(ix, ELLIPSOID_INTERSECTION_INDEX_LATITUDE_MIN + 0, bottomConeIntersection.xy);
        setIntersectionPair(ix, ELLIPSOID_INTERSECTION_INDEX_LATITUDE_MIN + 1, bottomConeIntersection.zw);
    #endif

    // Top cone
    #if defined(ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MAX_UNDER_HALF)
        vec4 topConeIntersection = intersectFlippedCone(flippedRay, u_ellipsoidRenderLatitudeCosSqrHalfMinMax.y);
        setIntersectionPair(ix, ELLIPSOID_INTERSECTION_INDEX_LATITUDE_MAX + 0, topConeIntersection.xy);
        setIntersectionPair(ix, ELLIPSOID_INTERSECTION_INDEX_LATITUDE_MAX + 1, topConeIntersection.zw);
    #elif defined(ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MAX_EQUAL_HALF)
        RayShapeIntersection topConeIntersection = intersectZPlane(ray);
        setShapeIntersection(ix, ELLIPSOID_INTERSECTION_INDEX_LATITUDE_MAX, topConeIntersection);
    #elif defined(ELLIPSOID_HAS_RENDER_BOUNDS_LATITUDE_MAX_OVER_HALF)
        RayShapeIntersection topConeIntersection = intersectRegularCone(ray, u_ellipsoidRenderLatitudeCosSqrHalfMinMax.y);
        setShapeIntersection(ix, ELLIPSOID_INTERSECTION_INDEX_LATITUDE_MAX, topConeIntersection);
    #endif

    // Wedge
    #if defined(ELLIPSOID_HAS_RENDER_BOUNDS_LONGITUDE_RANGE_EQUAL_ZERO)
        vec4 wedgeIntersect = intersectHalfPlane(ray, u_ellipsoidRenderLongitudeMinMax.x);
        setIntersectionPair(ix, ELLIPSOID_INTERSECTION_INDEX_LONGITUDE + 0, wedgeIntersect.xy);
        setIntersectionPair(ix, ELLIPSOID_INTERSECTION_INDEX_LONGITUDE + 1, wedgeIntersect.zw);
    #elif defined(ELLIPSOID_HAS_RENDER_BOUNDS_LONGITUDE_RANGE_UNDER_HALF)
        vec2 wedgeIntersect = intersectRegularWedge(ray, u_ellipsoidRenderLongitudeMinMax.x, u_ellipsoidRenderLongitudeMinMax.y);
        setIntersectionPair(ix, ELLIPSOID_INTERSECTION_INDEX_LONGITUDE, wedgeIntersect);
    #elif defined(ELLIPSOID_HAS_RENDER_BOUNDS_LONGITUDE_RANGE_EQUAL_HALF)
        vec2 wedgeIntersect = intersectHalfSpace(ray, u_ellipsoidRenderLongitudeMinMax.x);
        setIntersectionPair(ix, ELLIPSOID_INTERSECTION_INDEX_LONGITUDE, wedgeIntersect);
    #elif defined(ELLIPSOID_HAS_RENDER_BOUNDS_LONGITUDE_RANGE_OVER_HALF)
        vec4 wedgeIntersect = intersectFlippedWedge(ray, u_ellipsoidRenderLongitudeMinMax.x, u_ellipsoidRenderLongitudeMinMax.y);
        setIntersectionPair(ix, ELLIPSOID_INTERSECTION_INDEX_LONGITUDE + 0, wedgeIntersect.xy);
        setIntersectionPair(ix, ELLIPSOID_INTERSECTION_INDEX_LONGITUDE + 1, wedgeIntersect.zw);
    #endif
}
