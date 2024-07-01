void calcCov3D(vec3 scale, vec4 rot, float mod, out float[6] cov3D)
{
    mat3 S = mat3(
        mod * scale[0], 0, 0,
        0, mod * scale[1], 0,
        0, 0, mod * scale[2]
    );

    float r = rot.x;
    float x = rot.y;
    float y = rot.z;
    float z = rot.w;

    // Compute rotation matrix from quaternion
    mat3 R = mat3(
        1. - 2. * (y * y + z * z), 2. * (x * y - r * z), 2. * (x * z + r * y),
        2. * (x * y + r * z), 1. - 2. * (x * x + z * z), 2. * (y * z - r * x),
        2. * (x * z - r * y), 2. * (y * z + r * x), 1. - 2. * (x * x + y * y)
    );

    mat3 M = S * R;
    mat3 Sigma = transpose(M) * M;

    //we only need part of it, symmetric
    cov3D = float[6](
        Sigma[0][0], Sigma[0][1], Sigma[0][2],
        Sigma[1][1], Sigma[1][2], Sigma[2][2]
    );

}

vec3 calcCov2D(vec3 worldPos, float focal_x, float focal_y, float tan_fovx, float tan_fovy, float[6] cov3D, mat4 viewmatrix) {
    vec4 t = viewmatrix * vec4(worldPos, 1.0);

    float limx = 1.3 * tan_fovx;
    float limy = 1.3 * tan_fovy;
    float txtz = t.x / t.z;
    float tytz = t.y / t.z;
    t.x = min(limx, max(-limx, txtz)) * t.z;
    t.y = min(limy, max(-limy, tytz)) * t.z;

    mat3 J = mat3(
        focal_x / t.z, 0, -(focal_x * t.x) / (t.z * t.z),
        0, focal_y / t.z, -(focal_y * t.y) / (t.z * t.z),
        0, 0, 0
    );

    mat3 W = mat3(viewmatrix);
    mat3 T = W * J;
    mat3 Vrk = mat3(
        cov3D[0], cov3D[1], cov3D[2],
        cov3D[1], cov3D[3], cov3D[4],
        cov3D[2], cov3D[4], cov3D[5]
    );

    mat3 cov = transpose(T) * transpose(Vrk) * T;

    cov[0][0] += .3;
    cov[1][1] += .3;
    return vec3(cov[0][0], cov[0][1], cov[1][1]);
}

void gaussianSplatStage(ProcessedAttributes attributes, inout vec4 positionClip) {
    mat4 viewMatrix = czm_inverseView;
    mat4 projMatrix = czm_projection;

    vec4 clipPosition = czm_modelViewProjection * vec4(a_splatPosition,1.0);
    positionClip = clipPosition;

    float[6] cov3D;
    calcCov3D(attributes.scale, attributes.rotation, 5.0, cov3D);

    float aspect = czm_viewport.z / czm_viewport.w;
    float tan_fovx = 1./projMatrix[0][0];
    float tan_fovy = 1./(projMatrix[1][1] * aspect);
    float focal_x = czm_viewport.z * projMatrix[0][0] / 2.;

    // Compute 2D screen-space covariance matrix
    vec3 cov = calcCov2D(a_splatPosition, focal_x, focal_x, tan_fovx, tan_fovy, cov3D, viewMatrix);

    float mid = (cov.x + cov.z) / 2.0;
    float radius = length(vec2((cov.x - cov.z) / 2.0, cov.y));
    float lambda1 = mid + radius, lambda2 = mid - radius;

    if(lambda2 < 0.0) return;
    vec2 diagonalVector = normalize(vec2(cov.y, lambda1 - cov.x));
    vec2 majorAxis = min(sqrt(2.0 * lambda1), 1024.0) * diagonalVector;
    vec2 minorAxis = min(sqrt(2.0 * lambda2), 1024.0) * vec2(diagonalVector.y, -diagonalVector.x);

    vec2 corner = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2) - 1.;
    corner *= 2.0;

    vec2 deltaScreenPos = (corner.x * majorAxis + corner.y * minorAxis) * 2.0 / czm_viewport.zw;
    positionClip.xy += deltaScreenPos * positionClip.w;
    v_vertPos = corner;
}
