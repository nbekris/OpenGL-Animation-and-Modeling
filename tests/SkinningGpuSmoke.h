#pragma once
#include <cmath>
#include <fstream>
#include <iostream>

// Included by framework.cpp; run with --skinning-smoke from CS560Framework.
inline void RunSkinningGpuSmoke(Scene& scene)
{
    auto require = [](bool condition, const char* message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    };
    const GLuint program = scene.gBufferProgram->programId;
    const char* outputs[] = {"worldPos", "normalVec"};
    glTransformFeedbackVaryings(program, 2, outputs, GL_INTERLEAVED_ATTRIBS);
    glLinkProgram(program);
    GLint linked = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    require(linked != 0, "Skinning transform feedback shader link failed");
    glUseProgram(program);
    const glm::mat4 identity(1.0f);
    for (const char* name : {"WorldView", "WorldProj", "ModelTr"})
    {
        glUniformMatrix4fv(glGetUniformLocation(program, name), 1, GL_FALSE, &identity[0][0]);
    }
    glm::mat4 rigid(1.0f);
    rigid[3][0] = 5;
    glUniformMatrix4fv(glGetUniformLocation(program, "MeshNodeTr"), 1, GL_FALSE, &rigid[0][0]);
    std::vector<glm::mat4> palette(MAX_SKIN_BONES, identity);
    palette[0][3][0] = 4;
    palette[1][3][0] = 8;
    // A 90-degree rotation about Z for position and normal verification.
    palette[2][0] = glm::vec4(0,1,0,0);
    palette[2][1] = glm::vec4(-1,0,0,0);
    // Opposing rotations produce a singular blend for the last vertex.
    palette[3][0][0] = -1;
    palette[3][1][1] = -1;
    GLuint vao = 0, buffers[2] = {};
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    glGenBuffers(2, buffers);
    glBindBuffer(GL_UNIFORM_BUFFER, buffers[0]);
    glBufferData(GL_UNIFORM_BUFFER, palette.size() * sizeof(glm::mat4), palette.data(), GL_STATIC_DRAW);
    glUniformBlockBinding(program, glGetUniformBlockIndex(program, "SkinPalette"), 0);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, buffers[0]);
    glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER, buffers[1]);
    glBufferData(GL_TRANSFORM_FEEDBACK_BUFFER, 5 * 7 * sizeof(float), nullptr, GL_STREAM_READ);
    glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, buffers[1]);
    glVertexAttrib4f(0, 2, 0, 0, 1);
    glVertexAttrib3f(1, 1, 0, 0);
    glEnable(GL_RASTERIZER_DISCARD);
    glBeginTransformFeedback(GL_POINTS);
    auto point = [](int a, int b, float wa, float wb)
    {
        glVertexAttribI4i(4, a, b, 0, 0);
        glVertexAttrib4f(5, wa, wb, 0, 0);
        glDrawArrays(GL_POINTS, 0, 1);
    };
    point(0, 0, 1, 0);
    point(0, 1, 0.25f, 0.75f);
    point(0, 0, 0, 0);
    point(2, 0, 1, 0);
    point(0, 3, 0.5f, 0.5f);
    glEndTransformFeedback();
    glDisable(GL_RASTERIZER_DISCARD);
    float result[35] = {};
    glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, sizeof(result), result);
    for (float value : result)
    {
        require(std::isfinite(value), "Nonfinite GPU skinning output");
    }
    require(std::abs(result[0] - 6) < 1e-5, "GPU single bone deformation");
    require(std::abs(result[7] - 9) < 1e-5, "GPU two bone blend");
    require(std::abs(result[14] - 7) < 1e-5, "GPU unweighted rigid fallback");
    require(std::abs(result[21]) < 1e-5 && std::abs(result[22] - 2) < 1e-5,
        "GPU rotated vertex");
    require(std::abs(result[25]) < 1e-5 && std::abs(result[26] - 1) < 1e-5,
        "GPU rotated normal");
    glBindVertexArray(0);
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(2, buffers);
    require(glGetError() == GL_NO_ERROR, "GPU skinning OpenGL error");

    scene.mesh = true;
    scene.bone = true;
    Mesh* cachedRoman = scene.loadedModels[0].get();
    Mesh* cachedViking = scene.loadedModels[1].get();
    Mesh egyptian;
    for (const char* asset : {"roman_D", "egyptian_B", "viking_C", "roman_D"})
    {
        if (std::string(asset) == "egyptian_B")
        {
            egyptian.LoadMesh(std::string("fbx/fbx/") + asset + ".fbx");
            scene.model = &egyptian;
        }
        else
        {
            const int index = std::string(asset) == "viking_C" ? 1 : 0;
            require(scene.SelectModel(index), "Model selection failed");
            require(scene.model == (index == 0 ? cachedRoman : cachedViking),
                "Model selection must reuse the cached mesh");
            require(scene.selectedModel == index && scene.selectedAnimationPreset == 0,
                "Model selection state did not reset");
            require(scene.model->GetAnimator().IsPlaying(), "Selected model did not start playing");
        }
        auto& animator = scene.model->GetAnimator();
        animator.Pause();
        std::vector<float> first;
        const double duration = scene.model->GetAnimationData().clips.at(0).durationTicks;
        for (int frame = 0; frame < 2; ++frame)
        {
            require(animator.SetPlaybackRange(frame == 0 ? AnimationRange{0, duration * 0.5} : AnimationRange{duration * 0.5, duration}),
                "Asset smoke playback range");
            scene.DrawScene();
            std::vector<float> pixels(scene.width * scene.height * 4);
            glBindTexture(GL_TEXTURE_2D, scene.gBuffer.textures[0]);
            glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, pixels.data());
            size_t covered = 0;
            for (size_t i = 0; i < pixels.size(); i += 4)
            {
                for (int c = 0; c < 4; ++c)
                {
                    require(std::isfinite(pixels[i + c]), "Nonfinite asset G-buffer position");
                }
                if (pixels[i] != 0 || pixels[i+1] != 0 || pixels[i+2] != 0)
                {
                    ++covered;
                }
            }
            require(covered > 100, "Skinned asset produced no visible geometry");
            if (std::string(asset) == "roman_D")
            {
                std::vector<float> albedo(pixels.size());
                glBindTexture(GL_TEXTURE_2D, scene.gBuffer.textures[2]);
                glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, albedo.data());
                std::set<unsigned int> colors;
                for (size_t p = 0; p < pixels.size(); p += 4)
                {
                    if (pixels[p] == 0 && pixels[p+1] == 0 && pixels[p+2] == 0)
                    {
                        continue;
                    }
                    unsigned int color = 0;
                    for (int c = 0; c < 3; ++c)
                    {
                        require(std::isfinite(albedo[p+c]), "Nonfinite textured albedo");
                        color = (color << 8) | static_cast<unsigned int>(glm::clamp(albedo[p+c], 0.0f, 1.0f) * 255.0f);
                    }
                    colors.insert(color);
                }
                std::cout << "Roman textured albedo: " << colors.size() << " distinct colors\n";
                require(colors.size() > 32, "Roman diffuse texture produced no surface color variation");
            }
            if (frame == 0)
            {
                first = pixels;
            }
            else
            {
                require(first != pixels, "Animation did not change rendered geometry");
            }
            glBindTexture(GL_TEXTURE_2D, scene.gBuffer.textures[1]);
            glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, pixels.data());
            for (float value : pixels)
            {
                require(std::isfinite(value), "Nonfinite asset G-buffer normal");
            }
            // DrawScene evaluates the pose again while paused; the mesh must stay frozen.
            scene.DrawScene();
            glBindTexture(GL_TEXTURE_2D, scene.gBuffer.textures[0]);
            glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, pixels.data());
            if (frame == 0)
            {
                require(pixels == first, "Paused skinning pose changed");
            }
            require(glGetError() == GL_NO_ERROR, "Asset skinning OpenGL error");
            std::cout << asset << " frame " << frame << ": " << covered << " geometry pixels\n";
        }
        // Save an RGB readback for optional visual QA, bottom row first.
        std::vector<unsigned char> rgb(scene.width * scene.height * 3);
        glReadBuffer(GL_BACK);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, scene.width, scene.height, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
        std::ofstream image(std::string("../tests/build/") + asset + "-skinned.rgb", std::ios::binary);
        image.write(reinterpret_cast<const char*>(rgb.data()), rgb.size());
    }
    std::cout << "GPU LBS and asset smoke tests passed\n";
}
