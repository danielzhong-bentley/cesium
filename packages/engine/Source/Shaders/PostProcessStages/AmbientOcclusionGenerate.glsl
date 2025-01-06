precision highp float;

uniform sampler2D randomTexture;
uniform sampler2D depthTexture;
uniform float intensity;
uniform int kernelRadius;
uniform int KERNEL_SIZE;

vec3 generateSampleKernel(int index, int kernelSize) {
    float u1 = fract(sin(float(index) * 12.9898) * 43758.5453); // Pseudo-random using sin/hash
    float u2 = fract(sin(float(index) * 78.233) * 12345.6789);

    float r = sqrt(u1);                // Radius
    float theta = 2.0 * 3.1415926 * u2; // Angle

    vec3 kernelSample;
    kernelSample.x = r * cos(theta);
    kernelSample.y = r * sin(theta);
    kernelSample.z = sqrt(1.0 - u1); // Project to hemisphere

    // Scale the kernelSample so closer kernelSample contribute more
    float scale = float(index + 1) / float(kernelSize);
    scale = scale * scale; // Non-linear falloff
    kernelSample *= scale;

    return kernelSample;
}


vec4 pixelToEye(vec2 screenCoordinate)
{
    vec2 uv = screenCoordinate / czm_viewport.zw;
    float depth = czm_readDepth(depthTexture, uv);
    vec2 xy = 2.0 * uv - vec2(1.0);
    vec4 posEC = czm_inverseProjection * vec4(xy, depth, 1.0);
    posEC = posEC / posEC.w;
    // Avoid numerical error at far plane
    if (depth >= 1.0) {
        posEC.z = czm_currentFrustum.y;
    }
    return posEC;
}

float getDepth(vec2 screenPosition) {
    vec2 uv = screenPosition / czm_viewport.zw;
    float depth = czm_readDepth(depthTexture, uv);
    return clamp(depth, 0.0, 1.0);
}

float getViewZ(float depth) {
    vec2 uv = gl_FragCoord.xy / czm_viewport.zw;
    vec2 xy = 2.0 * uv - vec2(1.0);
    vec4 clipPosition = vec4(xy, depth, 1.0);
    vec4 viewPosition = czm_inverseProjection * clipPosition; // Unproject to view space
    return viewPosition.z / viewPosition.w;
}

vec2 projectCameraPositionToScreenSpace(vec3 cameraPosition) {
    vec4 clipPosition = czm_projection * vec4(cameraPosition, 1.0);
    vec3 ndc = clipPosition.xyz / clipPosition.w; // Normalized Device Coordinates
    return vec2(0.5) * (ndc.xy + vec2(1.0)); // Convert NDC to UV coordinates (0 to 1 range)
}

vec3 sampleDepthToCameraPosition(vec2 uv, float depth) {
    // Calculate the view-space Z coordinate using the provided depth
    float viewZ = getViewZ(depth);
    
    // Reuse pixelToEye for the full view-space position reconstruction
    vec4 viewPosition = pixelToEye(uv * czm_viewport.zw);

    return viewPosition.xyz; // Return the view-space position
}

// Reconstruct surface normal in eye coordinates, avoiding edges
vec3 getNormalXEdge(vec3 positionEC) //getViewNormal
{
    // Find the 3D surface positions at adjacent screen pixels
    vec2 centerCoord = gl_FragCoord.xy;
    vec3 positionLeft = pixelToEye(centerCoord + vec2(-1.0, 0.0)).xyz;
    vec3 positionRight = pixelToEye(centerCoord + vec2(1.0, 0.0)).xyz;
    vec3 positionUp = pixelToEye(centerCoord + vec2(0.0, 1.0)).xyz;
    vec3 positionDown = pixelToEye(centerCoord + vec2(0.0, -1.0)).xyz;

    // Compute potential tangent vectors
    vec3 dx0 = positionEC - positionLeft;
    vec3 dx1 = positionRight - positionEC;
    vec3 dy0 = positionEC - positionDown;
    vec3 dy1 = positionUp - positionEC;

    // The shorter tangent is more likely to be on the same surface
    vec3 dx = length(dx0) < length(dx1) ? dx0 : dx1;
    vec3 dy = length(dy0) < length(dy1) ? dy0 : dy1;

    return normalize(cross(dx, dy));
}

const float sqrtTwoPi = sqrt(czm_twoPi);

float gaussian(float x, float standardDeviation) {
    float argument = x / standardDeviation;
    return exp(-0.5 * argument * argument) / (sqrtTwoPi * standardDeviation);
}

vec3 debugOcclusionColor(float occlusion) {
			if (occlusion < 0.2) {
				return vec3(0.0, 1.0, 0.0); // green for low occlusion
			} else if (occlusion < 0.4) {
				return vec3(0.0, 0.0, 1.0); // blue for medium-low occlusion
			} else if (occlusion < 0.6) {
				return vec3(1.0, 1.0, 0.0); // yellow for medium occlusion
			} else if (occlusion < 0.8) {
				return vec3(1.0, 0.5, 0.0); // orange for medium-high occlusion
			} else if (occlusion < 1.0){
				return vec3(1.0, 0.0, 0.0); // red for high occlusion
			} else if (occlusion == 1.0){
				return vec3(0.0, 0.0, 0.0); // black for high occlusion
			}
			else {
				return vec3(1.0, 1.0, 1.0);
			}
		}

void main(void)
{

    float depth = getDepth(gl_FragCoord.xy);
    // vec3 color = vec3(depth);
    // out_FragColor = vec4(color, 1.0);
    // return;

    if ( depth == 1.0 ) {
        out_FragColor = vec4( 1.0 );
    } else {
        float viewZ = getViewZ( depth );
        
        vec4 viewPosition = pixelToEye(gl_FragCoord.xy);
        vec3 viewNormal = getNormalXEdge(viewPosition.xyz);

        float Sx = (float(kernelRadius) / 2.0) / czm_viewport.z;
        float Sy = (float(kernelRadius) / 2.0) / czm_viewport.w;

        float kernelDiagonal = sqrt(Sx * Sx + Sy * Sy);
		
        vec2 uv = gl_FragCoord.xy / czm_viewport.zw; // Convert fragment coordinates to UV space
        vec2 noiseUV = uv * czm_viewport.zw / 1024.0; // Adjust UVs based on resolution

        vec3 random = 2.0 * (texture(randomTexture, noiseUV).rgb - 0.5);

        vec3 tangent = normalize( random - viewNormal * dot( random, viewNormal ) );
		vec3 bitangent = cross( viewNormal, tangent );

        mat3 kernelMatrix = mat3( tangent, bitangent, viewNormal );

        float AOScale = abs(viewZ) * kernelDiagonal * 4.0;
		float occlusion = 0.0;

        for ( int i = 0; i < KERNEL_SIZE; i ++ ) {

			vec3 sampleVector = kernelMatrix * generateSampleKernel(i, KERNEL_SIZE);
			vec3 samplePoint = viewPosition.xyz + (sampleVector * AOScale);

			if (length(sampleVector - samplePoint) > AOScale * 0.05) {

				vec2 samplePointUv = projectCameraPositionToScreenSpace(samplePoint);

				float sampleDepth = getDepth( samplePointUv );
					
				vec3 sampleViewPosition = sampleDepthToCameraPosition(samplePointUv, sampleDepth);

				float viewDistance = length( sampleViewPosition - viewPosition.xyz ) / AOScale;
						
				vec3 sampleDirection = normalize(sampleViewPosition - viewPosition.xyz);
				float lightIntensity = clamp(dot(sampleDirection, normalize(viewNormal)), 0.0, 1.0);
				float distanceFadeout = clamp(1.0 - (viewDistance - 0.0) / 3.0, 0.0, 1.0);

				float sampleOcclusion = lightIntensity * distanceFadeout  / float(KERNEL_SIZE);
        		occlusion += sampleOcclusion;

               
			}
		}
        float innerRadius = AOScale;
		float outerRadius = innerRadius * 3.0;
				
		float kHigherOcclusion = 1.0;
		occlusion = clamp(occlusion / kHigherOcclusion, 0.0, 1.0);
		occlusion = pow(1.0 - occlusion, intensity);

        out_FragColor = vec4(vec3(occlusion), 1.0);	 

        vec3 debugColor = debugOcclusionColor(occlusion);
		// out_FragColor = vec4( vec3( debugColor ), 1.0 );
    }
}
