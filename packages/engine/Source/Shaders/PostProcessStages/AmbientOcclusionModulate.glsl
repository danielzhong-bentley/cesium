uniform sampler2D colorTexture;
uniform sampler2D ambientOcclusionTexture;
uniform bool ambientOcclusionOnly;
uniform bool enableBlur;
uniform sampler2D noiseTexture;
in vec2 v_textureCoordinates;

void main(void)
{
    vec4 color = texture(colorTexture, v_textureCoordinates);
    vec4 ao = texture(ambientOcclusionTexture, v_textureCoordinates);

    if (enableBlur) {
        vec4 blurredAO = vec4(0.0);
        float blurScale = 0.001;
        int blurSampleCount = 3;
        float weightSum = 0.0;

        // Base noise for consistent randomness
        vec3 baseNoise = 2.0 * (texture(noiseTexture, v_textureCoordinates).xyz - 0.5);

        for (int i = -blurSampleCount; i <= blurSampleCount; i++) {
            for (int j = -blurSampleCount; j <= blurSampleCount; j++) {
                vec2 sampleOffset = vec2(float(i), float(j)) * blurScale;

                // Add jitter to the sample offset using noise texture
                vec3 sampleNoise = texture(noiseTexture, v_textureCoordinates + sampleOffset).xyz;
                vec2 jitter = (vec2(baseNoise.x, baseNoise.y) + vec2(sampleNoise.x, sampleNoise.y)) * blurScale;
                vec2 finalOffset = sampleOffset + jitter;

                // Weighted sample
                float weight = 1.0 / (1.0 + length(sampleOffset)); // Distance-based weight
                blurredAO += texture(ambientOcclusionTexture, v_textureCoordinates + finalOffset) * weight;
                weightSum += weight;
            }
        }

        // Normalize the accumulated AO values
        if (weightSum > 0.0) {
            blurredAO /= weightSum;
        }

        ao = blurredAO;
    }

    out_FragColor = ambientOcclusionOnly ? ao : ao * color;
}