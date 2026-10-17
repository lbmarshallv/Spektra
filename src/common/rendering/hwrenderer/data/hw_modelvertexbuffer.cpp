/*
** hw_modelvertexbuffer.cpp
**
** hardware renderer model handling code
**
**---------------------------------------------------------------------------
**
** Copyright 2005-2020 Christoph Oelckers
** Copyright 2017-2025 GZDoom Maintainers and Contributors
** Copyright 2025-2026 UZDoom Maintainers and Contributors
**
** SPDX-License-Identifier: GPL-3.0-or-later
**
**---------------------------------------------------------------------------
**
*/


#include <float.h>
#include <cmath>
#include "v_video.h"
#include "cmdlib.h"
#include "hw_modelvertexbuffer.h"

//===========================================================================
//
//
//
//===========================================================================

FModelVertexBuffer::FModelVertexBuffer(bool needindex, bool singleframe)
{
	mVertexBuffer = screen->CreateVertexBuffer();
	mIndexBuffer = needindex ? screen->CreateIndexBuffer() : nullptr;

	static const FVertexBufferAttribute format[] = {
		{ 0, VATTR_VERTEX, VFmt_Float3, (int)myoffsetof(FModelVertex, x) },
		{ 0, VATTR_TEXCOORD, VFmt_Float2, (int)myoffsetof(FModelVertex, u) },
		{ 0, VATTR_NORMAL, VFmt_Packed_A2R10G10B10, (int)myoffsetof(FModelVertex, packedNormal) },
		{ 0, VATTR_LIGHTMAP, VFmt_Float3, (int)myoffsetof(FModelVertex, lu) },
		{ 0, VATTR_BONESELECTOR, VFmt_Byte4_UInt, (int)myoffsetof(FModelVertex, boneselector[0])},
		{ 0, VATTR_BONEWEIGHT, VFmt_Byte4, (int)myoffsetof(FModelVertex, boneweight[0]) },
		{ 1, VATTR_VERTEX2, VFmt_Float3, (int)myoffsetof(FModelVertex, x) },
		{ 1, VATTR_NORMAL2, VFmt_Packed_A2R10G10B10, (int)myoffsetof(FModelVertex, packedNormal) }
	};
	mVertexBuffer->SetFormat(2, 8, sizeof(FModelVertex), format);
}

//===========================================================================
//
//
//
//===========================================================================

FModelVertexBuffer::~FModelVertexBuffer()
{
	if (mIndexBuffer) delete mIndexBuffer;
	delete mVertexBuffer;
}

//===========================================================================
//
//
//
//===========================================================================

FModelVertex *FModelVertexBuffer::LockVertexBuffer(unsigned int size)
{
	mMapped = static_cast<FModelVertex*>(mVertexBuffer->Lock(size * sizeof(FModelVertex)));
	if (mMapped == nullptr || size == 0) return mMapped;
	mStaging.Resize(size);
	memset((void*)mStaging.Data(), 0, size * sizeof(FModelVertex));
	return mStaging.Data();
}

//===========================================================================
//
//
//
//===========================================================================

void FModelVertexBuffer::UnlockVertexBuffer()
{
	if (mMapped != nullptr && mStaging.Size() > 0)
	{
		memcpy((void*)mMapped, mStaging.Data(), mStaging.Size() * sizeof(FModelVertex));

		float mins[3] = { FLT_MAX, FLT_MAX, FLT_MAX };
		float maxs[3] = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
		bool any = false, weighted = false;
		for (const FModelVertex &v : mStaging)
		{
			if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) continue;
			const float p[3] = { v.x, v.y, v.z };
			for (int i = 0; i < 3; i++)
			{
				if (p[i] < mins[i]) mins[i] = p[i];
				if (p[i] > maxs[i]) maxs[i] = p[i];
			}
			any = true;
			if (v.boneweight[0] | v.boneweight[1] | v.boneweight[2] | v.boneweight[3]) weighted = true;
		}
		if (any)
		{
			for (int i = 0; i < 3; i++) { boundsMin[i] = mins[i]; boundsMax[i] = maxs[i]; }
			skinned = weighted;
			boundsValid = true;
		}
		mStaging.Reset();
	}
	mMapped = nullptr;
	mVertexBuffer->Unlock();
}

//===========================================================================
//
//
//
//===========================================================================

unsigned int *FModelVertexBuffer::LockIndexBuffer(unsigned int size)
{
	if (mIndexBuffer) return static_cast<unsigned int*>(mIndexBuffer->Lock(size * sizeof(unsigned int)));
	else return nullptr;
}

//===========================================================================
//
//
//
//===========================================================================

void FModelVertexBuffer::UnlockIndexBuffer()
{
	if (mIndexBuffer) mIndexBuffer->Unlock();
}
