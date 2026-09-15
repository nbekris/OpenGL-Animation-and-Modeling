# OpenGL Animation and Modeling

A simple graphics framework project for loading and rendering animated FBX character assets with OpenGL. The application uses a deferred-rendering pipeline and includes tools for inspecting a model's bind-pose skeleton.

The project currently opens a Roman character by default, renders it over a wireframe floor, and provides an ImGui control panel. Sample Viking and Egyptian FBX assets are also included for experimentation.

## Features

- OpenGL 3.3 core-profile renderer backed by GLFW and glbinding
- Assimp-based FBX mesh, bone, normal, and UV import
- GPU vertex skinning data (up to four bone influences per vertex)
- Deferred rendering with a four-texture G-buffer and a Phong lighting pass
- Wireframe floor and optional cyan bind-pose skeleton overlay
- Dear ImGui interface with playback, mesh, skeleton, model, and animation controls
- Camera navigation through mouse and numeric keypad controls

> **Current state:** The ImGui model and animation selectors are UI scaffolding; the renderer currently loads `roman_D.fbx` and draws its bind pose. The Play, Draw Mesh, and animation selection controls are not yet connected to playback or asset switching.

## Requirements

- Windows 10 or later
- Visual Studio with the Desktop development with C++ workload
- A compatible OpenGL 3.3+ graphics driver

The required headers and static libraries for GLFW, glbinding, Assimp, GLM, and Dear ImGui are included in the repository. The Visual Studio project is configured for `x64` and uses the `v145` platform toolset; if that toolset is not installed, retarget the project to an installed C++ toolset.

## Rendering pipeline

1. The selected FBX asset is imported with Assimp, triangulated, and uploaded to OpenGL vertex/index buffers.
2. The geometry pass writes world position, normal, diffuse, and specular data to the G-buffer.
3. A fullscreen quad reads those textures and applies Phong lighting.
4. The depth buffer is copied back to the default framebuffer so the floor and optional skeleton overlay compose correctly with the model.
