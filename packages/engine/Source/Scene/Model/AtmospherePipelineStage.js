import Cartesian3 from "../../Core/Cartesian3.js";
import CesiumMath from "../../Core/Math.js";
import defined from "../../Core/defined.js";
import ShaderDestination from "../../Renderer/ShaderDestination.js";
import AtmosphereStageFS from "../../Shaders/Model/AtmosphereStageFS.js";
import AtmosphereStageVS from "../../Shaders/Model/AtmosphereStageVS.js";

/**
 * The atmosphere pipeline stage applies all earth atmosphere effects that apply
 * to models, including fog.
 *
 * @namespace AtmospherePipelineStage
 *
 * @private
 */
const AtmospherePipelineStage = {
  name: "AtmospherePipelineStage", // Helps with debugging
};

AtmospherePipelineStage.process = function (
  renderResources,
  model,
  frameState
) {
  const shaderBuilder = renderResources.shaderBuilder;

  shaderBuilder.addDefine("HAS_ATMOSPHERE", undefined, ShaderDestination.BOTH);
  shaderBuilder.addDefine(
    "COMPUTE_POSITION_WC_ATMOSPHERE",
    undefined,
    ShaderDestination.BOTH
  );

  shaderBuilder.addVarying("vec3", "v_atmosphereRayleighColor");
  shaderBuilder.addVarying("vec3", "v_atmosphereMieColor");
  shaderBuilder.addVarying("float", "v_atmosphereOpacity");

  shaderBuilder.addVertexLines([AtmosphereStageVS]);
  shaderBuilder.addFragmentLines([AtmosphereStageFS]);

  // Add a uniform so fog is only calculated when the efcfect would
  // be non-negligible For example when the camera is in space, fog density decreases
  // to 0 so fog shouldn't be rendered. Since this state may change rapidly if
  // the camera is moving, this is implemented as a uniform, not a define.
  shaderBuilder.addUniform("bool", "u_isInFog", ShaderDestination.FRAGMENT);
  renderResources.uniformMap.u_isInFog = function () {
    const fogRenderable = frameState.fog.enabled && frameState.fog.renderable;
    if (!fogRenderable) {
      return false;
    }

    // We only need a rough measure of distance to the model, so measure
    // from the camera to the bounding sphere center.
    const distance = Cartesian3.distance(
      frameState.camera.positionWC,
      model.boundingSphere.center
    );

    return (
      CesiumMath.fog(distance, frameState.fog.density) > CesiumMath.EPSILON3
    );
  };

  // When the
  shaderBuilder.addUniform(
    "bool",
    "u_perFragmentGroundAtmosphere",
    ShaderDestination.BOTH
  );
  renderResources.uniformMap.u_perFragmentGroundAtmosphere = function () {
    if (!defined(frameState.atmosphere)) {
      return false;
    }

    const cameraDistance = Cartesian3.magnitude(frameState.camera.positionWC);
    const nightFadeStart = frameState.atmosphere.nightFadeRange.x;
    return cameraDistance > nightFadeStart;
  };

  // Only apply the color correction math in the shader if one of the
  // HSB color shift options are set.
  shaderBuilder.addUniform(
    "bool",
    "u_shouldColorCorrect",
    ShaderDestination.FRAGMENT
  );
  renderResources.uniformMap.u_shouldColorCorrect = function () {
    const atmosphere = frameState.atmosphere;
    if (!defined(atmosphere)) {
      return false;
    }

    return !(
      CesiumMath.equalsEpsilon(atmosphere.hueShift, 0.0, CesiumMath.EPSILON7) &&
      CesiumMath.equalsEpsilon(
        atmosphere.saturationShift,
        0.0,
        CesiumMath.EPSILON7
      ) &&
      CesiumMath.equalsEpsilon(
        atmosphere.brightnessShift,
        0.0,
        CesiumMath.EPSILON7
      )
    );
  };
};

export default AtmospherePipelineStage;
