import ShaderDestination from "../../Renderer/ShaderDestination.js";
import GaussianSplatVS from "../../Shaders/Model/GaussianSplatVS.js";
import GaussianSplatFS from "../../Shaders/Model/GaussianSplatFS.js";
import Pass from "../../Renderer/Pass.js";
import PrimitiveType from "../../Core/PrimitiveType.js";
import BlendingState from "../BlendingState.js";

const GaussianSplatTexturePipelineStage = {
  name: "GaussianSplatTexturePipelineStage",
};

GaussianSplatTexturePipelineStage.process = function (
  renderResources,
  primitive,
  frameState,
) {
  const { shaderBuilder } = renderResources;

  const renderStateOptions = renderResources.renderStateOptions;
  renderStateOptions.cull.enabled = false;
  renderStateOptions.depthMask = false;
  renderStateOptions.depthTest.enabled = false;
  renderStateOptions.blending = BlendingState.PRE_MULTIPLIED_ALPHA_BLEND;

  renderResources.alphaOptions.pass = Pass.GAUSSIAN_SPLATS;

  shaderBuilder.addDefine(
    "HAS_GAUSSIAN_SPLATS",
    undefined,
    ShaderDestination.BOTH,
  );

  shaderBuilder.addDefine(
    "HAS_SPLAT_TEXTURE",
    undefined,
    ShaderDestination.BOTH,
  );

  //shaderBuilder.addAttribute("vec2", "a_screenQuadPosition");
  shaderBuilder.addAttribute("float", "a_splatIndex");

  shaderBuilder.addVarying("vec4", "v_splatColor");
  shaderBuilder.addVarying("vec2", "v_vertPos");

  shaderBuilder.addUniform(
    "highp usampler2D",
    "u_splatAttributeTexture",
    ShaderDestination.VERTEX,
  );

  shaderBuilder.addUniform("mat4", "u_scalingMatrix", ShaderDestination.VERTEX);

  shaderBuilder.addUniform("float", "u_splatScale", ShaderDestination.VERTEX);

  const uniformMap = renderResources.uniformMap;

  uniformMap.u_splatScale = function () {
    return renderResources.model?.style?.splatScale ?? 1.0;
  };

  uniformMap.u_splatAttributeTexture = function () {
    return primitive.gaussianSplatTexture;
  };

  uniformMap.u_scalingMatrix = function () {
    return renderResources.model.sceneGraph.components.nodes[0].matrix;
  };

  renderResources.instanceCount = renderResources.count;
  renderResources.count = 4;
  renderResources.primitiveType = PrimitiveType.TRIANGLE_STRIP;

  shaderBuilder.addVertexLines(GaussianSplatVS);
  shaderBuilder.addFragmentLines(GaussianSplatFS);
};

export default GaussianSplatTexturePipelineStage;
