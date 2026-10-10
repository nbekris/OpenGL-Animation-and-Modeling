#pragma once
#include "Animator.h"

struct AnimationPreset
{
    const char* name;
    AnimationRange range;
};

// Tick ranges for the current character asset, with exclusive end boundaries.
static const AnimationPreset AnimationPresets[] =
{
    { "Cycle",     {   0.0, 515.0 } },
    { "Idle",      {   0.0, 340.0 } }, // Provisional idle range.
    { "Run",       { 339.5, 362.0 } },
    { "Fall Down", { 364.0, 442.0 } },
    { "Attack",    { 443.0, 515.0 } }
};
