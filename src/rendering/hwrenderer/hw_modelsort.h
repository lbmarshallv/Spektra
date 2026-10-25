/*
** hw_modelsort.h
**
** Pre-sorted triangle orders for SEMITRANSPARENT models (hw_modelsort.cpp).
**
**---------------------------------------------------------------------------
**
** SPDX-License-Identifier: GPL-3.0-or-later
**
**---------------------------------------------------------------------------
*/

#pragma once

#include "vectors.h"

class FModel;
class IIndexBuffer;

constexpr int MODELSORT_DIRECTIONS = 10;	// 8 around, above, below

// Builds the sorted orders the first time it's called (main thread, before drawing).
void UpdateSortedModels();
void DeleteSortedModels();

// The order to use for a thing seen from 'eye': its centre and yaw (radians, map convention).
int ModelSortDirection(const DVector3 &eye, const DVector3 &center, double yawRadians);
int ModelSortDirection(double relYaw, double elevation);

// If this draw call of a model frame has pre-sorted orders: the index buffer and the first index for 'direction'.
bool ModelSortedDraw(FModel *model, unsigned frameBase, bool arrays, unsigned first, unsigned count, int direction, IIndexBuffer *&ib, unsigned &start);
