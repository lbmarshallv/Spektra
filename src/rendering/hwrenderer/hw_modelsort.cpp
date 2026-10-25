/*
** hw_modelsort.cpp
**
** Pre-sorted triangle orders for SEMITRANSPARENT models.
**
** Blended triangles must be drawn back to front, but a model draws its
** triangles in file order. Sorting every model's triangles every frame is
** far too slow for forests and would break instancing, so instead each
** model frame used by a SEMITRANSPARENT MODELDEF entry is sorted once, from
** 10 directions: 8 around it (45 degrees apart, like the rotations of a
** Doom sprite), one from above and one from below. When the blended pass
** draws it, each thing picks the order for the direction it's seen from
** (ModelSortDirection) and the draw uses that index list instead of the
** model's own. Things using the same direction can still share instanced
** draws.
**
** The sorting is done once, before the first frame is drawn, spread over
** all CPU cores. All lists go into one static index buffer.
**
** This is exact for a thing seen from one of the 10 directions from far
** away, and close otherwise. Faces that cut through each other have no
** right order at all.
**
**---------------------------------------------------------------------------
**
** Copyright 2026 UZDoom Maintainers and Contributors
**
** SPDX-License-Identifier: GPL-3.0-or-later
**
**---------------------------------------------------------------------------
*/

#include <algorithm>
#include <unordered_map>
#include <vector>
#include <math.h>
#include "actor.h"
#include "info.h"
#include "g_levellocals.h"
#include "models.h"
#include "model.h"
#include "modelrenderer.h"
#include "i_modelvertexbuffer.h"
#include "texturemanager.h"
#include "v_video.h"
#include "c_cvars.h"
#include "c_dispatch.h"
#include "printf.h"
#include "i_time.h"
#include "stats.h"
#include "parallel_for.h"
#include "hwrenderer/data/buffers.h"
#include "hw_modelsort.h"

CVARD(Bool, r_semitransparent_sort, true, CVAR_ARCHIVE | CVAR_GLOBALCONFIG, "SEMITRANSPARENT models: draw their soft pixels with triangles pre-sorted back to front for the direction they're seen from")

namespace
{
	struct FSortKey
	{
		FModel *model;
		unsigned frameBase;
		unsigned first;		// index offset (DrawElements) or first vertex (DrawArrays), relative to frameBase
		unsigned count;
		bool arrays;

		bool operator==(const FSortKey &o) const
		{
			return model == o.model && frameBase == o.frameBase && first == o.first && count == o.count && arrays == o.arrays;
		}
	};

	struct FSortKeyHash
	{
		size_t operator()(const FSortKey &k) const
		{
			size_t h = std::hash<const void *>()(k.model);
			h ^= (size_t(k.frameBase) * 0x9e3779b97f4a7c15ull) + (h << 6) + (h >> 2);
			h ^= (size_t(k.first) * 0xc2b2ae3d27d4eb4full) + (h << 6) + (h >> 2);
			h ^= (size_t(k.count) * 0x165667b19e3779f9ull) + (h << 6) + (h >> 2) + (k.arrays ? 1 : 0);
			return h;
		}
	};

	// One draw call of a captured model frame.
	struct FCapturedDraw
	{
		FSortKey key;
		unsigned vertexStart;		// into FCapturedFrame::positions: the vertex at frameBase
		TArray<unsigned> indices;	// the triangle list (relative to frameBase), as the model draws it
		unsigned outFirst = 0;		// where its 10 sorted lists go in the shared index buffer
	};

	struct FCapturedFrame
	{
		TArray<FVector3> positions;	// map coordinates (x, y, height), yaw 0
		TArray<FCapturedDraw> draws;
	};

	// Captures the draws a model frame makes. The vertex buffer is filled by the model's own code, exactly as for the GPU,
	// so the frame offsets and index ranges recorded here are the ones the hardware renderer will be given.
	class FSortCaptureBuffer : public IModelVertexBuffer
	{
	public:
		TArray<FModelVertex> verts;
		TArray<unsigned int> indices;

		FModelVertex *LockVertexBuffer(unsigned int size) override
		{
			verts.Resize(size);
			if (size > 0) memset((void *)verts.Data(), 0, size * sizeof(FModelVertex));
			return verts.Data();
		}
		void UnlockVertexBuffer() override {}
		unsigned int *LockIndexBuffer(unsigned int size) override
		{
			indices.Resize(size);
			return indices.Data();
		}
		void UnlockIndexBuffer() override {}
	};

	class FSortCaptureRenderer : public FModelRenderer
	{
	public:
		FCapturedFrame *out = nullptr;
		VSMatrix matrix;
		FModel *model = nullptr;
		unsigned frameBase = 0;

		ModelRendererType GetType() const override { return CaptureModelRendererType; }
		void BeginDrawModel(FRenderStyle, int, const VSMatrix &, bool) override {}
		void EndDrawModel(FRenderStyle, int) override {}
		IModelVertexBuffer *CreateVertexBuffer(bool, bool) override { return new FSortCaptureBuffer; }
		VSMatrix GetViewToWorldMatrix() override { VSMatrix m; m.loadIdentity(); return m; }
		void BeginDrawHUDModel(FRenderStyle, const VSMatrix &, bool, int) override {}
		void EndDrawHUDModel(FRenderStyle, int) override {}
		void SetInterpolation(double) override {}
		void SetMaterial(FGameTexture *, bool, FTranslationID, AActor *) override {}
		void SetupFrame(FModel *m, unsigned int frame1, unsigned int, unsigned int, int) override
		{
			model = m;
			frameBase = frame1;
		}

		FSortCaptureBuffer *Buffer() const
		{
			return model ? static_cast<FSortCaptureBuffer *>(model->GetVertexBuffer(CaptureModelRendererType)) : nullptr;
		}

		// The first n vertices from frameBase on, transformed (each draw keeps its own copy).
		unsigned Positions(unsigned n)
		{
			auto vb = Buffer();
			const unsigned start = out->positions.Size();
			for (unsigned v = frameBase; v < frameBase + n && v < vb->verts.Size(); v++)
			{
				const FModelVertex &mv = vb->verts[v];
				float in[4] = { mv.x, mv.y, mv.z, 1.f }, res[4];
				matrix.multMatrixPoint(in, res);
				out->positions.Push(FVector3(res[0], res[2], res[1]));	// renderer (x, height, y) -> map (x, y, height)
			}
			return start;
		}

		void DrawArrays(int start, int count) override
		{
			auto vb = Buffer();
			if (vb == nullptr || count < 3 || frameBase + start + count > vb->verts.Size()) return;
			FCapturedDraw d;
			d.key = { model, frameBase, (unsigned)start, (unsigned)count, true };
			d.vertexStart = Positions(start + count);
			d.indices.Resize(count - count % 3);
			for (unsigned i = 0; i < d.indices.Size(); i++) d.indices[i] = start + i;
			out->draws.Push(std::move(d));
		}

		void DrawElements(int numIndices, size_t offset) override
		{
			auto vb = Buffer();
			const unsigned first = unsigned(offset / sizeof(unsigned int));
			if (vb == nullptr || numIndices < 3 || first + numIndices > vb->indices.Size()) return;
			FCapturedDraw d;
			d.key = { model, frameBase, first, (unsigned)numIndices, false };
			d.indices.Resize(numIndices - numIndices % 3);
			const unsigned limit = vb->verts.Size() - frameBase;
			unsigned needed = 1;
			for (unsigned i = 0; i < d.indices.Size(); i++)
			{
				const unsigned idx = vb->indices[first + i];
				d.indices[i] = idx < limit ? idx : 0;
				needed = max(needed, d.indices[i] + 1);
			}
			d.vertexStart = Positions(needed);
			out->draws.Push(std::move(d));
		}
	};

	std::unordered_map<FSortKey, unsigned, FSortKeyHash> sortedDraws;	// -> first index of direction 0 in sortedIB
	IIndexBuffer *sortedIB = nullptr;
	bool sortBuilt = false;
	unsigned sortedTriangles = 0, sortedModels = 0;
	double sortBuildMS = 0;

	// View directions (from the camera towards the model) in yaw-0 map space: 8 around, then from above and from below.
	FVector3 SortDirection(int d)
	{
		if (d == MODELSORT_DIRECTIONS - 2) return FVector3(0, 0, -1);
		if (d == MODELSORT_DIRECTIONS - 1) return FVector3(0, 0, 1);
		const double a = d * (2 * M_PI / 8);
		return FVector3((float)-cos(a), (float)-sin(a), 0.f);
	}
}

//==========================================================================
//
// Which of the pre-sorted orders fits a thing seen from this direction.
// relYaw: the angle from the thing to the eye minus the thing's yaw
// (radians, map convention); elevation: the eye's angle above the thing.
//
//==========================================================================

int ModelSortDirection(double relYaw, double elevation)
{
	if (elevation > M_PI * (50. / 180.)) return MODELSORT_DIRECTIONS - 2;
	if (elevation < -M_PI * (50. / 180.)) return MODELSORT_DIRECTIONS - 1;
	int k = (int)floor(relYaw * (8. / (2 * M_PI)) + 0.5);
	k %= 8;
	if (k < 0) k += 8;
	return k;
}

int ModelSortDirection(const DVector3 &eye, const DVector3 &center, double yawRadians)
{
	const DVector3 d = eye - center;
	const double flat = sqrt(d.X * d.X + d.Y * d.Y);
	return ModelSortDirection(atan2(d.Y, d.X) - yawRadians, atan2(d.Z, flat));
}

//==========================================================================
//
// The sorted list for one of a model's draw calls, if it has one.
//
//==========================================================================

bool ModelSortedDraw(FModel *model, unsigned frameBase, bool arrays, unsigned first, unsigned count, int direction, IIndexBuffer *&ib, unsigned &start)
{
	if (sortedIB == nullptr || direction < 0 || direction >= MODELSORT_DIRECTIONS || !r_semitransparent_sort) return false;
	auto it = sortedDraws.find(FSortKey{ model, frameBase, first, count, arrays });
	if (it == sortedDraws.end()) return false;
	ib = sortedIB;
	start = it->second + direction * (count - count % 3);
	return true;
}

//==========================================================================
//
// Builds every sorted list, once, before the first frame. Capturing runs
// on this thread (the models' code isn't thread-safe); the sorting is
// spread over all cores, one job per draw call and direction.
//
//==========================================================================

void UpdateSortedModels()
{
	if (sortBuilt) return;
	FLevelLocals *Level = primaryLevel;
	if (Level == nullptr || Level->info == nullptr) return;	// pixel stretch comes from the level
	sortBuilt = true;
	if (screen == nullptr || screen->Backend() == 0) return;	// GLES: no SEMITRANSPARENT sorting

	const uint64_t startTime = I_nsTime();

	// Which model frames: every SEMITRANSPARENT MODELDEF frame, and the LODModel models of classes whose spawn frame is one.
	struct FRequest { int modelID; int frame; FSpriteModelFrame *smf; unsigned index; };
	TArray<FRequest> requests;
	auto addRequest = [&](int modelID, int frame, FSpriteModelFrame *smf, unsigned index)
	{
		if (modelID < 0 || modelID >= (int)Models.Size() || frame < 0) return;
		for (auto &r : requests) if (r.modelID == modelID && r.frame == frame) return;
		requests.Push({ modelID, frame, smf, index });
	};
	for (auto &smf : SpriteModelFrames)
	{
		if (!(smf.getFlags(nullptr) & MDL_SEMITRANSPARENT)) continue;
		for (unsigned i = 0; i < smf.modelsAmount && i < smf.modelIDs.Size(); i++) addRequest(smf.modelIDs[i], smf.modelframes[i], &smf, i);
	}
	for (auto cls : PClassActor::AllActorClasses)
	{
		if (cls->ActorInfo()->LODmodels == nullptr) continue;
		FState *state = GetDefaultByType(cls)->SpawnState;
		FSpriteModelFrame *smf = state ? FindModelFrame(cls, state->sprite, state->GetFrame(), false) : nullptr;
		if (smf == nullptr || smf->modelsAmount == 0 || !(smf->getFlags(nullptr) & MDL_SEMITRANSPARENT)) continue;
		for (const LODmodel *m = cls->ActorInfo()->LODmodels; m != nullptr; m = m->Next)
		{
			if (m->type == LOD_Model) addRequest(m->modelID, smf->modelframes[0], smf, 0);
		}
	}
	if (requests.Size() == 0) return;

	// Capture (this thread).
	TArray<FCapturedFrame> frames;
	frames.Resize(requests.Size());
	for (unsigned r = 0; r < requests.Size(); r++)
	{
		const FRequest &req = requests[r];
		FSpriteModelFrame *smf = req.smf;
		FModel *mdl = Models[req.modelID];
		FSortCaptureRenderer cap;
		cap.out = &frames[r];
		cap.matrix = smf->ObjectToWorldMatrix(Level, DVector3(0, 0, 0), DRotator(nullAngle, nullAngle, nullAngle), DVector2(1, 1), smf->getFlags(nullptr), 0.);
		const bool hadBuffer = mdl->GetVertexBuffer(CaptureModelRendererType) != nullptr;
		mdl->BuildVertexBuffer(&cap);
		cap.model = mdl;
		const unsigned i = req.index;
		FGameTexture *tex = i < smf->skinIDs.Size() && smf->skinIDs[i].isValid() ? TexMan.GetGameTexture(smf->skinIDs[i], true) : nullptr;
		const FTextureID *ssid = (i * MD3_MAX_SURFACES) < smf->surfaceskinIDs.Size() ? &smf->surfaceskinIDs[i * MD3_MAX_SURFACES] : nullptr;
		mdl->RenderFrame(&cap, tex, req.frame, req.frame, 0., NO_TRANSLATION, ssid, 0, nullptr);	// bone base 0: bind pose, no bone upload
		if (!hadBuffer)
		{
			delete mdl->GetVertexBuffer(CaptureModelRendererType);
			mdl->SetVertexBuffer(CaptureModelRendererType, nullptr);
		}
	}

	// Lay out the shared index buffer: per draw, MODELSORT_DIRECTIONS lists one after the other.
	struct FJob { FCapturedFrame *frame; FCapturedDraw *draw; int direction; };
	TArray<FJob> jobs;
	unsigned total = 0;
	for (auto &f : frames)
	{
		for (auto &d : f.draws)
		{
			if (sortedDraws.count(d.key)) continue;	// the same draw reached through another request
			d.outFirst = total;
			total += d.indices.Size() * MODELSORT_DIRECTIONS;
			sortedDraws[d.key] = d.outFirst;
			sortedTriangles += d.indices.Size() / 3;
			for (int dir = 0; dir < MODELSORT_DIRECTIONS; dir++) jobs.Push({ &f, &d, dir });
		}
	}
	sortedModels = requests.Size();
	if (total == 0) return;
	TArray<unsigned> out(total, true);

	// Sort (all cores): far triangles first, by their centre's distance along the view direction.
	parallel_for((int)jobs.Size(), [&](int j)
	{
		const FJob &job = jobs[j];
		const FCapturedDraw &d = *job.draw;
		const FVector3 v = SortDirection(job.direction);
		const unsigned tris = d.indices.Size() / 3;
		std::vector<std::pair<float, unsigned>> order(tris);
		const FVector3 *pos = &job.frame->positions[d.vertexStart];
		for (unsigned t = 0; t < tris; t++)
		{
			const FVector3 &a = pos[d.indices[t * 3]], &b = pos[d.indices[t * 3 + 1]], &c = pos[d.indices[t * 3 + 2]];
			const float depth = (a.X + b.X + c.X) * v.X + (a.Y + b.Y + c.Y) * v.Y + (a.Z + b.Z + c.Z) * v.Z;
			order[t] = { depth, t };
		}
		std::stable_sort(order.begin(), order.end(), [](const std::pair<float, unsigned> &x, const std::pair<float, unsigned> &y) { return x.first > y.first; });
		unsigned *dst = &out[d.outFirst + job.direction * d.indices.Size()];
		for (unsigned t = 0; t < tris; t++)
		{
			const unsigned src = order[t].second * 3;
			dst[t * 3] = d.indices[src];
			dst[t * 3 + 1] = d.indices[src + 1];
			dst[t * 3 + 2] = d.indices[src + 2];
		}
	});

	sortedIB = screen->CreateIndexBuffer();
	sortedIB->SetData(out.Size() * sizeof(unsigned), out.Data(), BufferUsageType::Static);
	sortBuildMS = (I_nsTime() - startTime) / 1e6;
	DPrintf(DMSG_NOTIFY, "SEMITRANSPARENT: sorted %u triangles of %u model frames from %d directions in %.1f ms\n", sortedTriangles, sortedModels, MODELSORT_DIRECTIONS, sortBuildMS);
}

void DeleteSortedModels()
{
	delete sortedIB;
	sortedIB = nullptr;
	sortedDraws.clear();
	sortBuilt = false;
	sortedTriangles = sortedModels = 0;
}

CCMD(semitransparent_sortinfo)
{
	if (sortedIB == nullptr) Printf("No pre-sorted SEMITRANSPARENT models%s.\n", sortBuilt ? "" : " yet");
	else Printf("SEMITRANSPARENT: %u model frames, %zu draw calls, %u triangles sorted from %d directions in %.1f ms (%.1f MB of indices), r_semitransparent_sort %s\n",
		sortedModels, sortedDraws.size(), sortedTriangles, MODELSORT_DIRECTIONS, sortBuildMS, sortedTriangles * 3. * MODELSORT_DIRECTIONS * 4. / (1024 * 1024), *r_semitransparent_sort ? "on" : "off");
}
