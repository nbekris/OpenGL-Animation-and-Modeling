# OpenGL Animation and Modeling

A simple graphics framework project for loading and rendering animated FBX character assets with OpenGL. The application uses a deferred-rendering pipeline and includes tools for inspecting a model's bind-pose skeleton.

The project currently opens a Roman character by default, renders it over a wireframe floor, and provides an ImGui control panel. Sample Viking and Egyptian FBX assets are also included for experimentation.

## Features

- OpenGL 3.3 core-profile renderer backed by GLFW and glbinding
- Assimp-based FBX mesh, bone, normal, and UV import
- GPU linear blend skinning, with Assimp limiting and normalizing influences to four per vertex
- Deferred rendering with a four-texture G-buffer and a Phong lighting pass
- Wireframe floor and optional red animated skeleton overlay
- Dear ImGui interface with playback, mesh, skeleton, model, and animation controls
- Camera navigation through mouse and numeric keypad controls

> **Current state:** The renderer loads `roman_D.fbx` and starts animation playback. Enable **Draw Mesh** to view the skinned surface alongside the skeleton. Playback and animation preset controls are connected; the model selector remains scaffolding. The Viking asset currently fails the existing uniform-scale import requirement at its `shield` node.

## Requirements

- Windows 10 or later
- Visual Studio with the Desktop development with C++ workload
- A compatible OpenGL 3.3+ graphics driver

The required headers and static libraries for GLFW, glbinding, Assimp, GLM, and Dear ImGui are included in the repository. The Visual Studio project is configured for `x64` and uses the `v145` platform toolset; if that toolset is not installed, retarget the project to an installed C++ toolset.

## Rendering pipeline

1. Assimp imports and triangulates the FBX asset, limits bone influences to four per vertex, and normalizes retained weights.
2. Animator evaluates the root-relative pose. Each submesh retains its own inverse bind matrices and uploads a palette of up to 128 bone matrices through a uniform buffer.
3. The geometry shader deforms positions and normals using linear blend skinning. Unweighted vertices use their mesh-node transform. The geometry pass writes world position, normal, diffuse, and specular data to the G-buffer.
4. A fullscreen quad reads those textures and applies Phong lighting.
5. The depth buffer is copied back to the default framebuffer so the floor and skeleton overlay compose with the model.
