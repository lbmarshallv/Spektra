/*
** hw_lodsprites.cpp
**
** LOD sprites: things with +LODSPRITE (foliage clusters and actors) are
** drawn as simple 2D sprites of their model when they're far away:
**
**   - past the line cull distance, where the world LOD shows the outdoor
**     level (hw_worldlod.cpp), and
**   - from their LODModel <distance>, "Sprite" level on, if they have one.
**
** A LODModel <distance>, "None" level (or DistanceCheck) hides them past that.
**
** The sprites are made by the engine: the first time a class is needed
** (on the first frame of a level, or when a new LODSPRITE thing shows up),
** its model is rendered on the CPU from 8 angles, 45 degrees apart, like the
** 8 rotations of a Doom sprite: flat, unshaded texture colours, a few
** colours, low resolution. The 8 frames go into one small texture, shared by
** every thing of that class in every level until the texture manager is reset.
**
** Drawing never walks the BSP:
**
**   - Cluster plants are in a static vertex buffer built once per level:
**     4 vertices per plant holding its base, size and yaw. The vertex shader
**     (main.vp) turns each into a quad facing the view plane, picks the frame
**     for the angle it's seen from, and drops plants outside the class's
**     distance window, so a plant switches between model and sprite exactly
**     at its distance. Plants are grouped in map squares for the frustum and
**     distance tests and ordered so that plants of one class and light share
**     long runs: a few draws per class and light.
**   - LODSPRITE actors are kept in a per-level list (AActor::LinkLodSprite);
**     their quads are written each frame.
**
** Sprites hidden behind walls are drawn too (depth testing hides them): the
** cost is a few vertices each, much less than finding out which are hidden.
**
** Light, colour and fog are the sector's (or 3D-floor slice's) at the plant,
** looked up every frame like foliage. There are no dynamic lights on sprites.
** The renderer's model path skips a thing past its sprite distance, using the
** same test (HWDrawInfo::LodSpriteWindow), so a model and its sprite are never
** drawn together.
**
** Only the main perspective view draws LOD sprites. Portals, mirrors and
** skyboxes keep the old behaviour (a "Sprite" level hides foliage there).
** Needs OpenGL or Vulkan (not GLES).
**
**---------------------------------------------------------------------------
**
** Copyright 2026 UZDoom Maintainers and Contributors
**
** SPDX-License-Identifier: GPL-3.0-or-later
**
**---------------------------------------------------------------------------
*/

#include <float.h>
#include <math.h>
#include <algorithm>
#include <tuple>
#include "p_foliage.h"
#include "actor.h"
#include "info.h"
#include "d_main.h"
#include "g_levellocals.h"
#include "r_defs.h"
#include "models.h"
#include "model.h"
#include "modelrenderer.h"
#include "i_modelvertexbuffer.h"
#include "texturemanager.h"
#include "image.h"
#include "bitmap.h"
#include "v_video.h"
#include "c_cvars.h"
#include "c_dispatch.h"
#include "printf.h"
#include "stats.h"
#include "i_time.h"
#include "hw_material.h"
#include "hw_renderstate.h"
#include "hw_cvars.h"
#include "hwrenderer/scene/hw_drawinfo.h"
#include "flatvertices.h"
#include "buffers.h"

CVARD(Bool, gl_lodsprites, true, CVAR_ARCHIVE | CVAR_GLOBALCONFIG, "Draw +LODSPRITE foliage and actors as generated 2D sprites past the line cull distance and from their LODModel \"Sprite\" distance")
CVARD(Int, gl_lodsprite_size, 64, CVAR_ARCHIVE | CVAR_GLOBALCONFIG, "Size in pixels of the larger side of each generated LOD sprite frame (16-256). Changing it makes the sprites again.")
CVARD(Int, gl_lodsprite_colors, 16, CVAR_ARCHIVE | CVAR_GLOBALCONFIG, "Colours per generated LOD sprite, shared by its 8 angles (2-256, 0 = keep every colour). Changing it makes the sprites again.")
CVARD(Float, gl_lodsprite_alpha, 0.5f, CVAR_ARCHIVE | CVAR_GLOBALCONFIG, "Alpha cutoff for LOD sprites: lower keeps thin leaves visible from farther away")
CVARD(Float, gl_lodsprite_fade, 256.f, CVAR_ARCHIVE | CVAR_GLOBALCONFIG, "Width in map units of the dithered cross-fade between a LODSPRITE model and its sprite. 0 = switch at once.")
CVARD(Float, gl_lodsprite_mipalpha, 0.25f, CVAR_ARCHIVE | CVAR_GLOBALCONFIG, "Alpha added to LOD sprites per mipmap level, so thin leaves and needles stay solid in the distance. 0 = off.")
CVARD(Bool, gl_lodsprite_planes, true, CVAR_ARCHIVE | CVAR_GLOBALCONFIG, "LOD sprite foliage as 4 crossed planes fixed to the plant (turned, tilted and scaled like its model), each side showing the view from its direction, instead of one camera-facing sprite. Needs model instancing (OpenGL 4.3 or Vulkan).")
CVARD(Int, gl_lodsprite_chunk, 2048, CVAR_ARCHIVE | CVAR_GLOBALCONFIG, "Size of the map squares LOD sprite foliage is grouped in for the frustum and distance tests. Changing it rebuilds the batch.")

EXTERN_CVAR(Float, r_lod)
EXTERN_CVAR(Int, r_distance_cull_type)
EXTERN_CVAR(Bool, gl_render_foliage)
EXTERN_CVAR(Bool, gl_render_things)
EXTERN_CVAR(Bool, r_debug_disable_vis_filter)
EXTERN_CVAR(Float, gl_mask_threshold)

static constexpr int LODSPRITE_ANGLES = 8;		// must match main.vp
static constexpr int LODSPRITE_SUPERSAMPLE = 4;	// samples per pixel side when rendering a frame
static constexpr int LODSPRITE_BORDER = 1;
static constexpr double LODSPRITE_PLANE_ANGLE = 67.5;	// planes: sides turned less than this from the eye are shown (2 or 3 of them)		// empty pixels around each frame, so filtering never picks up its neighbour

//==========================================================================
//
// Statistics
//
//==========================================================================

static struct FLodSpriteStats
{
	int units, draws, chunks, actors, actorDraws;
	cycle_t time;
} lodStats;

static double lodGenMS = 0;		// generating sprites, last time any were made
static int lodGenCount = 0;		// classes generated since the game started
static double lodBuildMS = 0;	// building the last foliage batch

void ResetLodSpriteStats()
{
	lodStats.units = lodStats.draws = lodStats.chunks = lodStats.actors = lodStats.actorDraws = 0;
	lodStats.time.Reset();
}

//==========================================================================
//
// One class's generated sprite: 8 frames side by side in one texture.
//
//==========================================================================

struct FLodSpriteImage
{
	FGameTexture *tex = nullptr;	// owned by the texture manager
	FTextureID texID;
	bool valid = false;
	int frameWidth = 0, frameHeight = 0;
	// Quad extents at the class's own scale, in map units around the thing's origin (border included).
	float halfWidth = 0, top = 0, bottom = 0;
	int colors = 0, triangles = 0;
	FString error;					// why it couldn't be made
};

// Every class made so far, by texture name (which holds the settings), so classes are made once per game, not per
// level. An entry only counts while the texture manager still has its texture.
static TMap<FString, FLodSpriteImage> lodImageCache;
static int lodImageGeneration = 0;	// bumped by lodsprite_regenerate

static FString LodSpriteTexName(PClassActor *cls)
{
	FString name;
	name.Format("@LODSPRITE_%d_%d_%d_%s", lodImageGeneration, clamp<int>(gl_lodsprite_size, 16, 256), *gl_lodsprite_colors <= 0 ? 0 : clamp<int>(gl_lodsprite_colors, 2, 256), cls->TypeName.GetChars());
	return name;
}

class FLodSpriteImageSource : public FImageSource
{
public:
	TArray<uint32_t> pixels;	// BGRA

	FLodSpriteImageSource(int w, int h)
	{
		Width = w;
		Height = h;
		pixels.Resize(w * h);
		memset(pixels.Data(), 0, pixels.Size() * sizeof(uint32_t));	// the border is never written: it must start transparent
	}

	int CopyPixels(FBitmap *bmp, int conversion, int frame = 0) override
	{
		bmp->CopyPixelDataRGB(0, 0, (const uint8_t *)pixels.Data(), Width, Height, 4, Width * 4, 0, CF_BGRA);
		return -1;
	}
};

//==========================================================================
//
// Capturing a model's triangles on the CPU. The models fill a vertex
// buffer through this renderer just as they do for the GPU, then "draw"
// into it; the draws are collected as triangles.
//
//==========================================================================

class FCaptureVertexBuffer : public IModelVertexBuffer
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

struct FSkinPixels
{
	TArray<uint32_t> pixels;	// BGRA
	int width = 0, height = 0;
};

struct FCapturedTri
{
	FVector3 p[3];		// map coordinates (x, y, height), class scale, yaw 0
	FVector2 uv[3];
	int skin;			// index into the skin list, -1 = none (drawn grey)
};

class FCaptureModelRenderer : public FModelRenderer
{
public:
	TArray<FCapturedTri> tris;
	TArray<FSkinPixels> skins;
	TMap<FGameTexture *, int> skinIndex;
	VSMatrix matrix;	// object to world (renderer coordinates)
	FModel *model = nullptr;
	unsigned frameBase = 0;
	int currentSkin = -1;

	ModelRendererType GetType() const override { return CaptureModelRendererType; }
	void BeginDrawModel(FRenderStyle, int, const VSMatrix &, bool) override {}
	void EndDrawModel(FRenderStyle, int) override {}
	IModelVertexBuffer *CreateVertexBuffer(bool, bool) override { return new FCaptureVertexBuffer; }
	VSMatrix GetViewToWorldMatrix() override { VSMatrix m; m.loadIdentity(); return m; }
	void BeginDrawHUDModel(FRenderStyle, const VSMatrix &, bool, int) override {}
	void EndDrawHUDModel(FRenderStyle, int) override {}
	void SetInterpolation(double) override {}

	void SetMaterial(FGameTexture *skin, bool, FTranslationID, AActor *) override
	{
		currentSkin = -1;
		if (skin == nullptr || !skin->isValid()) return;
		if (int *found = skinIndex.CheckKey(skin))
		{
			currentSkin = *found;
			return;
		}
		FSkinPixels sp;
		FBitmap bmp = skin->GetTexture()->GetBgraBitmap(nullptr);
		if (bmp.GetPixels() && bmp.GetWidth() > 0 && bmp.GetHeight() > 0)
		{
			sp.width = bmp.GetWidth();
			sp.height = bmp.GetHeight();
			sp.pixels.Resize(sp.width * sp.height);
			for (int y = 0; y < sp.height; y++)
			{
				memcpy(&sp.pixels[y * sp.width], bmp.GetPixels() + y * bmp.GetPitch(), sp.width * 4);
			}
		}
		currentSkin = skinIndex[skin] = skins.Push(std::move(sp));
	}

	void SetupFrame(FModel *m, unsigned int frame1, unsigned int, unsigned int, int) override
	{
		model = m;
		frameBase = frame1;
	}

	void AddTriangle(const FModelVertex *v0, const FModelVertex *v1, const FModelVertex *v2)
	{
		FCapturedTri t;
		const FModelVertex *v[3] = { v0, v1, v2 };
		for (int i = 0; i < 3; i++)
		{
			float in[4] = { v[i]->x, v[i]->y, v[i]->z, 1.f }, out[4];
			matrix.multMatrixPoint(in, out);
			t.p[i] = FVector3(out[0], out[2], out[1]);	// renderer (x, height, y) -> map (x, y, height)
			t.uv[i] = FVector2(v[i]->u, v[i]->v);
		}
		t.skin = currentSkin;
		tris.Push(t);
	}

	FCaptureVertexBuffer *Buffer() const
	{
		return model ? static_cast<FCaptureVertexBuffer *>(model->GetVertexBuffer(CaptureModelRendererType)) : nullptr;
	}

	void DrawArrays(int start, int count) override
	{
		auto vb = Buffer();
		if (vb == nullptr) return;
		for (int i = 0; i + 2 < count; i += 3)
		{
			const unsigned a = frameBase + start + i;
			if (a + 2 >= vb->verts.Size()) break;
			AddTriangle(&vb->verts[a], &vb->verts[a + 1], &vb->verts[a + 2]);
		}
	}

	void DrawElements(int numIndices, size_t offset) override
	{
		auto vb = Buffer();
		if (vb == nullptr) return;
		const unsigned first = unsigned(offset / sizeof(unsigned int));
		for (int i = 0; i + 2 < numIndices; i += 3)
		{
			if (first + i + 2 >= vb->indices.Size()) break;
			const unsigned a = frameBase + vb->indices[first + i], b = frameBase + vb->indices[first + i + 1], c = frameBase + vb->indices[first + i + 2];
			if (a >= vb->verts.Size() || b >= vb->verts.Size() || c >= vb->verts.Size()) continue;
			AddTriangle(&vb->verts[a], &vb->verts[b], &vb->verts[c]);
		}
	}
};

//==========================================================================
//
// Colour reduction: median cut over the opaque pixels of all 8 frames.
//
//==========================================================================

static void ReduceColors(TArray<uint32_t> &pixels, int maxColors, int &usedColors)
{
	TArray<uint32_t> colors;
	for (uint32_t c : pixels) if ((c >> 24) != 0) colors.Push(c & 0xffffff);
	usedColors = 0;
	if (colors.Size() == 0) return;

	struct Box { unsigned first, count; };
	TArray<Box> boxes;
	boxes.Push({ 0, colors.Size() });
	auto channel = [](uint32_t c, int ch) { return int((c >> (ch * 8)) & 255); };
	while ((int)boxes.Size() < maxColors)
	{
		// split the box with the widest channel range
		int best = -1, bestCh = 0, bestRange = 0;
		for (unsigned b = 0; b < boxes.Size(); b++)
		{
			if (boxes[b].count < 2) continue;
			for (int ch = 0; ch < 3; ch++)
			{
				int lo = 255, hi = 0;
				for (unsigned i = 0; i < boxes[b].count; i++)
				{
					const int v = channel(colors[boxes[b].first + i], ch);
					lo = min(lo, v);
					hi = max(hi, v);
				}
				if (hi - lo > bestRange) { bestRange = hi - lo; best = b; bestCh = ch; }
			}
		}
		if (best < 0) break;	// every box is a single colour
		Box &box = boxes[best];
		uint32_t *start = &colors[box.first];
		std::sort(start, start + box.count, [&](uint32_t a, uint32_t b) { return channel(a, bestCh) < channel(b, bestCh); });
		const unsigned half = box.count / 2;
		Box second = { box.first + half, box.count - half };
		box.count = half;
		boxes.Push(second);
	}

	TArray<uint32_t> palette;
	for (auto &box : boxes)
	{
		uint64_t sum[3] = { 0, 0, 0 };
		for (unsigned i = 0; i < box.count; i++)
		{
			for (int ch = 0; ch < 3; ch++) sum[ch] += channel(colors[box.first + i], ch);
		}
		uint32_t c = 0;
		for (int ch = 0; ch < 3; ch++) c |= uint32_t(sum[ch] / box.count) << (ch * 8);
		palette.Push(c);
	}
	usedColors = palette.Size();

	for (auto &p : pixels)
	{
		if ((p >> 24) == 0) continue;
		int bestDist = INT_MAX;
		uint32_t bestColor = 0;
		for (uint32_t c : palette)
		{
			int d = 0;
			for (int ch = 0; ch < 3; ch++)
			{
				const int diff = channel(p, ch) - channel(c, ch);
				d += diff * diff;
			}
			if (d < bestDist) { bestDist = d; bestColor = c; }
		}
		p = (p & 0xff000000) | bestColor;
	}
}

//==========================================================================
//
// Makes a class's sprite from the model of its spawn state: the full detail
// model at the class's own scale, seen from the side from 8 angles, flat
// texture colours (no shading), with an orthographic projection so the
// sprite can stand anywhere.
//
//==========================================================================

static bool GenerateLodSprite(FLevelLocals *Level, PClassActor *cls, FLodSpriteImage &out)
{
	const AActor *def = GetDefaultByType(cls);
	FState *state = def->SpawnState;
	FSpriteModelFrame *smf = state ? FindModelFrame(cls, state->sprite, state->GetFrame(), false) : nullptr;
	if (smf == nullptr || smf->modelsAmount == 0)
	{
		out.error = "no model for the spawn state";
		return false;
	}

	FCaptureModelRenderer cap;
	const unsigned smfFlags = smf->getFlags(nullptr);
	cap.matrix = smf->ObjectToWorldMatrix(Level, DVector3(0, 0, 0), DRotator(nullAngle, nullAngle, nullAngle), def->Scale, smfFlags, 0.);
	for (unsigned i = 0; i < smf->modelsAmount; i++)
	{
		const int modelid = smf->modelIDs[i];
		const int frame = smf->modelframes[i];
		if (modelid < 0 || modelid >= (int)Models.Size() || frame < 0) continue;
		FModel *mdl = Models[modelid];
		FGameTexture *tex = smf->skinIDs[i].isValid() ? TexMan.GetGameTexture(smf->skinIDs[i], true) : nullptr;
		const FTextureID *ssid = (i * MD3_MAX_SURFACES) < smf->surfaceskinIDs.Size() ? &smf->surfaceskinIDs[i * MD3_MAX_SURFACES] : nullptr;
		const bool hadBuffer = mdl->GetVertexBuffer(CaptureModelRendererType) != nullptr;
		mdl->BuildVertexBuffer(&cap);
		cap.model = mdl;
		cap.frameBase = 0;
		mdl->RenderFrame(&cap, tex, frame, frame, 0., NO_TRANSLATION, ssid, 0, nullptr);	// bone base 0: the bind pose, no bone upload
		if (!hadBuffer)
		{
			// The CPU copy is only needed now.
			delete mdl->GetVertexBuffer(CaptureModelRendererType);
			mdl->SetVertexBuffer(CaptureModelRendererType, nullptr);
		}
	}
	out.triangles = cap.tris.Size();
	if (cap.tris.Size() == 0)
	{
		out.error = "the model has no triangles with a texture";
		return false;
	}

	// Extent around the origin: the same width from every angle.
	float radius = 0, zmin = FLT_MAX, zmax = -FLT_MAX;
	for (auto &t : cap.tris)
	{
		for (auto &p : t.p)
		{
			radius = max(radius, sqrtf(p.X * p.X + p.Y * p.Y));
			zmin = min(zmin, p.Z);
			zmax = max(zmax, p.Z);
		}
	}
	if (radius <= 0 || zmax <= zmin)
	{
		out.error = "the model is flat";
		return false;
	}

	// Frame size: square pixels, the larger side gl_lodsprite_size.
	const int size = clamp<int>(gl_lodsprite_size, 16, 256);
	const float worldW = radius * 2, worldH = zmax - zmin;
	const int inner = size - 2 * LODSPRITE_BORDER;
	int innerW, innerH;
	if (worldH >= worldW) { innerH = inner; innerW = max(2, (int)ceilf(inner * worldW / worldH)); }
	else { innerW = inner; innerH = max(2, (int)ceilf(inner * worldH / worldW)); }
	const int W = innerW + 2 * LODSPRITE_BORDER, H = innerH + 2 * LODSPRITE_BORDER;
	const float texel = max(worldW / innerW, worldH / innerH);	// map units per pixel
	// Centre the content in the inner area.
	const float spanW = texel * innerW, spanH = texel * innerH;
	const float left = -spanW * 0.5f, topZ = (zmax + zmin) * 0.5f + spanH * 0.5f;

	auto img = new FLodSpriteImageSource(W * LODSPRITE_ANGLES, H);
	const int SS = LODSPRITE_SUPERSAMPLE;
	// SEMITRANSPARENT models also show their soft pixels, so those count as covering too.
	const uint32_t alphaCut = (smfFlags & MDL_SEMITRANSPARENT) ? 32 : 128;
	const int sw = innerW * SS, sh = innerH * SS;
	TArray<uint32_t> sampleColor(sw * sh, true);
	TArray<float> sampleDepth(sw * sh, true);

	for (int k = 0; k < LODSPRITE_ANGLES; k++)
	{
		// Seen from angle k * 45 degrees (counterclockwise from the front): look towards the origin.
		const double a = k * (2 * M_PI / LODSPRITE_ANGLES);
		const float rightX = (float)-sin(a), rightY = (float)cos(a);	// screen right
		const float lookX = (float)-cos(a), lookY = (float)-sin(a);		// into the screen
		for (unsigned i = 0; i < sampleColor.Size(); i++) { sampleColor[i] = 0; sampleDepth[i] = FLT_MAX; }

		for (auto &t : cap.tris)
		{
			float sx[3], sy[3], dz[3];
			for (int i = 0; i < 3; i++)
			{
				sx[i] = (t.p[i].X * rightX + t.p[i].Y * rightY - left) / spanW * sw;
				sy[i] = (topZ - t.p[i].Z) / spanH * sh;
				dz[i] = t.p[i].X * lookX + t.p[i].Y * lookY;
			}
			const float area = (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
			if (fabsf(area) < 1e-8f) continue;	// edge-on: no area (both sides of a triangle are drawn)
			const int x0 = max(0, (int)floorf(min(sx[0], min(sx[1], sx[2])))), x1 = min(sw - 1, (int)ceilf(max(sx[0], max(sx[1], sx[2]))));
			const int y0 = max(0, (int)floorf(min(sy[0], min(sy[1], sy[2])))), y1 = min(sh - 1, (int)ceilf(max(sy[0], max(sy[1], sy[2]))));
			const FSkinPixels *skin = t.skin >= 0 ? &cap.skins[t.skin] : nullptr;
			const bool hasPixels = skin && skin->pixels.Size() > 0;
			for (int y = y0; y <= y1; y++)
			{
				const float py = y + 0.5f;
				for (int x = x0; x <= x1; x++)
				{
					const float px = x + 0.5f;
					const float w0 = ((sx[1] - px) * (sy[2] - py) - (sx[2] - px) * (sy[1] - py)) / area;
					const float w1 = ((sx[2] - px) * (sy[0] - py) - (sx[0] - px) * (sy[2] - py)) / area;
					const float w2 = 1.f - w0 - w1;
					if (w0 < -1e-5f || w1 < -1e-5f || w2 < -1e-5f) continue;
					const float depth = w0 * dz[0] + w1 * dz[1] + w2 * dz[2];
					const int s = y * sw + x;
					if (depth >= sampleDepth[s]) continue;
					uint32_t c = 0xff808080;
					if (hasPixels)
					{
						float u = w0 * t.uv[0].X + w1 * t.uv[1].X + w2 * t.uv[2].X;
						float v = w0 * t.uv[0].Y + w1 * t.uv[1].Y + w2 * t.uv[2].Y;
						u -= floorf(u);
						v -= floorf(v);
						const int tx = min(skin->width - 1, (int)(u * skin->width));
						const int ty = min(skin->height - 1, (int)(v * skin->height));
						c = skin->pixels[ty * skin->width + tx];
						if ((c >> 24) < alphaCut) continue;	// alpha-tested like the model (leaves)
					}
					sampleDepth[s] = depth;
					sampleColor[s] = c | 0xff000000;
				}
			}
		}

		// Down to the frame's pixels: colour of the covered samples, alpha = coverage.
		uint64_t frameSum[3] = { 0, 0, 0 };
		unsigned frameCount = 0;
		for (int y = 0; y < innerH; y++)
		{
			for (int x = 0; x < innerW; x++)
			{
				unsigned sum[3] = { 0, 0, 0 }, covered = 0;
				for (int j = 0; j < SS; j++)
				{
					for (int i = 0; i < SS; i++)
					{
						const uint32_t c = sampleColor[(y * SS + j) * sw + x * SS + i];
						if ((c >> 24) == 0) continue;
						covered++;
						for (int ch = 0; ch < 3; ch++) sum[ch] += (c >> (ch * 8)) & 255;
					}
				}
				uint32_t out = 0;
				if (covered > 0)
				{
					for (int ch = 0; ch < 3; ch++)
					{
						out |= (sum[ch] / covered) << (ch * 8);
						frameSum[ch] += sum[ch] / covered;
					}
					// Alpha is twice the coverage: a pixel half covered by needles counts as solid, as the model looks
					// at that size (its own alpha test sees one texel at a time).
					out |= uint32_t(min<unsigned>(255, covered * 2 * 255 / (SS * SS))) << 24;
					frameCount++;
				}
				img->pixels[(y + LODSPRITE_BORDER) * img->GetWidth() + k * W + x + LODSPRITE_BORDER] = out;
			}
		}
		// Empty pixels take the frame's average colour, so filtering doesn't darken the edges.
		uint32_t fill = 0;
		if (frameCount > 0) for (int ch = 0; ch < 3; ch++) fill |= uint32_t(frameSum[ch] / frameCount) << (ch * 8);
		for (int y = 0; y < H; y++)
		{
			for (int x = 0; x < W; x++)
			{
				uint32_t &p = img->pixels[y * img->GetWidth() + k * W + x];
				if ((p >> 24) == 0) p = fill;
			}
		}
	}

	const int maxColors = *gl_lodsprite_colors <= 0 ? 0 : clamp<int>(gl_lodsprite_colors, 2, 256);
	out.colors = 0;
	if (maxColors > 0) ReduceColors(img->pixels, maxColors, out.colors);

	FString name = LodSpriteTexName(cls);
	out.tex = MakeGameTexture(new FImageTexture(img), name.GetChars(), ETextureType::Override);
	out.tex->SetUpscaleFlag(0, true);
	out.texID = TexMan.AddGameTexture(out.tex, true);
	out.frameWidth = W;
	out.frameHeight = H;
	out.halfWidth = W * texel * 0.5f;	// the quad covers the whole frame, border included
	out.top = topZ + LODSPRITE_BORDER * texel;
	out.bottom = topZ - spanH - LODSPRITE_BORDER * texel;
	out.valid = true;
	return true;
}

// Finds or makes a class's sprite.
static const FLodSpriteImage &GetLodSpriteImage(FLevelLocals *Level, PClassActor *cls)
{
	FString name = LodSpriteTexName(cls);
	FLodSpriteImage *found = lodImageCache.CheckKey(name);
	if (found != nullptr)
	{
		if (!found->valid) return *found;	// couldn't be made; tried again after lodsprite_regenerate
		FTextureID id = TexMan.CheckForTexture(name.GetChars(), ETextureType::Override);
		if (id.isValid() && id == found->texID && TexMan.GetGameTexture(id) == found->tex) return *found;
	}
	const uint64_t start = I_nsTime();
	FLodSpriteImage img;
	if (!GenerateLodSprite(Level, cls, img))
	{
		Printf(TEXTCOLOR_ORANGE "LODSPRITE: no sprite for %s: %s. It is drawn as a model.\n", cls->TypeName.GetChars(), img.error.GetChars());
	}
	const double ms = (I_nsTime() - start) / 1e6;
	lodGenMS = ms;
	lodGenCount++;
	DPrintf(DMSG_NOTIFY, "LODSPRITE: %s: %d triangles, %dx%d frames, %d colours in %.1f ms\n", cls->TypeName.GetChars(), img.triangles, img.frameWidth, img.frameHeight, img.colors, ms);
	lodImageCache[name] = img;
	return lodImageCache[name];
}

//==========================================================================
//
// Per level
//
//==========================================================================

// A run of plants in the batch sharing class, sector class, sector (and 3D-floor light slice) and map square.
struct FLodSpriteUnit
{
	unsigned firstPlant, numPlants;		// in batch order
	unsigned chunk;
	sector_t *sector;
	FVector3 pos;			// one of its plants (picks the 3D-floor light slice)
	uint16_t type;			// foliage type
	uint8_t secClass;		// LODSPRITE_SEC_*
};

struct FLodSpriteChunk
{
	float x1 = FLT_MAX, x2 = -FLT_MAX, y1 = FLT_MAX, y2 = -FLT_MAX, z1 = FLT_MAX, z2 = -FLT_MAX;
	bool visible = false;	// this frame
	double nearSq = 0, farSq = 0;
};

struct FLodSprites
{
	const sector_t *sectorsData = nullptr;
	unsigned numSectors = 0;
	int chunkSize = 0, generation = -1, size = 0, colors = 0;
	bool planes = false;			// gl_lodsprite_planes (and instancing) when built: crossed planes instead of billboards
	TArray<unsigned> imageMesh;		// planes: per image, its first vertex in vb (the class's plane mesh)
	unsigned firstInstance = 0;		// planes: the first plant record in vb
	TArray<uint8_t> sectorClass;	// per sector

	// Sprites of the classes this level uses (copies of the cache entries), by class.
	TMap<PClassActor *, int> classIndex;
	TArray<FLodSpriteImage> images;
	TArray<PClassActor *> pending;	// actor classes met while drawing, made after the frame's sprites are drawn
	bool actorsScanned = false;

	// Cluster plants
	FFoliageSystem *builtFoliage = nullptr;
	unsigned builtInstances = ~0u;
	TArray<int> typeImage;			// per foliage type: index into images, -1 = not drawn as a LOD sprite
	TArray<uint8_t> typeFarModel;	// per foliage type: LODPLANE (its model is drawn past the cull distance)
	TArray<uint32_t> farPlants;		// LODPLANE plants (instance indices)
	TArray<FLodSpriteUnit> units;
	TArray<FLodSpriteChunk> chunks;
	unsigned numPlants = 0;
	IVertexBuffer *vb = nullptr;
	IIndexBuffer *ib = nullptr;

	~FLodSprites() { FreeBuffers(); }

	void FreeBuffers()
	{
		delete vb;
		delete ib;
		vb = nullptr;
		ib = nullptr;
	}

	bool Matches(FLevelLocals *Level) const
	{
		return sectorsData == Level->sectors.Data() && numSectors == Level->sectors.Size() && chunkSize == clamp<int>(gl_lodsprite_chunk, 256, 65536)
			&& generation == lodImageGeneration && size == *gl_lodsprite_size && colors == *gl_lodsprite_colors && planes == WantPlanes();
	}

	static bool WantPlanes()
	{
		return gl_lodsprite_planes && screen->RenderState()->SupportsModelInstancing();
	}

	int ImageFor(FLevelLocals *Level, PClassActor *cls)
	{
		if (int *found = classIndex.CheckKey(cls)) return *found;
		const int index = images.Push(GetLodSpriteImage(Level, cls));
		classIndex[cls] = index;
		return index;
	}

	const FLodSpriteImage *Find(PClassActor *cls)
	{
		int *found = classIndex.CheckKey(cls);
		return found ? &images[*found] : nullptr;
	}

	void Reset(FLevelLocals *Level);
	void BuildBatch(FLevelLocals *Level);
};

void DeleteLodSprites(FLodSprites *sprites)
{
	delete sprites;
}

void FLodSprites::Reset(FLevelLocals *Level)
{
	sectorsData = Level->sectors.Data();
	numSectors = Level->sectors.Size();
	chunkSize = clamp<int>(gl_lodsprite_chunk, 256, 65536);
	generation = lodImageGeneration;
	size = gl_lodsprite_size;
	colors = gl_lodsprite_colors;
	planes = WantPlanes();

	sectorClass.Resize(numSectors);
	for (unsigned i = 0; i < numSectors; i++)
	{
		const sector_t *sec = &Level->sectors[i];
		uint8_t c = 0;
		if (HWSectorLodOnly(sec)) c |= LODSPRITE_SEC_LODONLY;
		if (!(sec->MoreFlags & SECMF_NOLOD) && HWSectorOutdoor(sec)) c |= LODSPRITE_SEC_MODEL;
		sectorClass[i] = c;
	}
	classIndex.Clear();
	images.Clear();
	pending.Clear();
	actorsScanned = false;
	builtFoliage = nullptr;
	builtInstances = ~0u;
	typeImage.Clear();
	typeFarModel.Clear();
	farPlants.Clear();
	units.Clear();
	chunks.Clear();
	numPlants = 0;
	FreeBuffers();
}

static int LightSlice(sector_t *sec, const FVector3 &pos)
{
	auto &list = sec->e->XFloor.lightlist;
	if (list.Size() == 0) return 0;
	const DVector2 xy(pos.X, pos.Y);
	const double z = pos.Z + 1.;
	unsigned slice = 0;
	while (slice + 1 < list.Size() && z < list[slice + 1].plane.ZatPoint(xy)) slice++;
	return slice;
}

static uint32_t Morton16(uint32_t x, uint32_t y)
{
	auto spread = [](uint32_t v)
	{
		v &= 0xffff;
		v = (v | (v << 8)) & 0x00ff00ff;
		v = (v | (v << 4)) & 0x0f0f0f0f;
		v = (v | (v << 2)) & 0x33333333;
		v = (v | (v << 1)) & 0x55555555;
		return v;
	};
	return spread(x) | (spread(y) << 1);
}

//==========================================================================
//
// The static batch of LODSPRITE cluster plants: 4 vertices per plant, in
// the order (sector class, class, sector light as built, map square,
// sector, light slice), so plants of a class lit alike are consecutive and
// visible squares make long runs.
//
//==========================================================================

void FLodSprites::BuildBatch(FLevelLocals *Level)
{
	const uint64_t start = I_nsTime();
	FFoliageSystem *foliage = Level->Foliage;
	builtFoliage = foliage;
	builtInstances = foliage ? foliage->Instances.Size() : 0;
	units.Clear();
	chunks.Clear();
	numPlants = 0;
	FreeBuffers();
	if (foliage == nullptr) return;

	typeImage.Resize(foliage->Types.Size());
	typeFarModel.Resize(foliage->Types.Size());
	farPlants.Clear();
	bool any = false, anyFar = false;
	for (unsigned t = 0; t < foliage->Types.Size(); t++)
	{
		typeImage[t] = -1;
		PClassActor *cls = foliage->Types[t].cls;
		const int perf = GetDefaultByType(cls)->perfFlags;
		typeFarModel[t] = !!(perf & PERF_LODPLANE);
		anyFar |= !!typeFarModel[t];
		if (!(perf & PERF_LODSPRITE) || (perf & PERF_LODPLANE)) continue;	// LODPLANE wins: it is its own stand-in
		const int img = ImageFor(Level, cls);
		if (images[img].valid)
		{
			typeImage[t] = img;
			any = true;
		}
	}
	if (anyFar)
	{
		for (unsigned i = 0; i < foliage->Instances.Size(); i++)
		{
			if (typeFarModel[foliage->Instances[i].type]) farPlants.Push(i);
		}
	}
	if (!any) return;

	// Sectors ranked by their light now, so sectors lit alike sit together.
	TArray<int> lightRank(numSectors, true);
	{
		TArray<int> order(numSectors, true);
		for (unsigned i = 0; i < numSectors; i++) order[i] = i;
		auto key = [&](int i)
		{
			const sector_t &s = Level->sectors[i];
			return std::make_tuple((int)s.lightlevel, (uint32_t)s.Colormap.LightColor.d, (uint32_t)s.Colormap.FadeColor.d, (int)s.Colormap.FogDensity,
				(uint32_t)s.SpecialColors[sector_t::sprites].d, (uint32_t)s.AdditiveColors[sector_t::sprites].d, i);
		};
		std::sort(order.begin(), order.end(), [&](int a, int b) { return key(a) < key(b); });
		for (unsigned i = 0; i < numSectors; i++) lightRank[order[i]] = i;
	}

	struct FPlant
	{
		uint8_t secClass;
		uint16_t type;
		int rank;
		uint32_t morton;
		unsigned chunk;
		int sector, slice;
		unsigned instance;
	};
	TArray<FPlant> plants;
	TMap<uint64_t, unsigned> chunkMap;
	for (unsigned i = 0; i < foliage->Instances.Size(); i++)
	{
		const FFoliageInstance &inst = foliage->Instances[i];
		const int img = typeImage[inst.type];
		if (img < 0) continue;
		const int cx = (int)floor(inst.pos.X / chunkSize), cy = (int)floor(inst.pos.Y / chunkSize);
		const uint64_t ckey = (uint64_t(uint32_t(cx)) << 32) | uint32_t(cy);
		unsigned chunk;
		if (unsigned *found = chunkMap.CheckKey(ckey)) chunk = *found;
		else chunkMap[ckey] = chunk = chunks.Reserve(1);

		const FLodSpriteImage &im = images[img];
		const float hw = im.halfWidth * inst.spriteScaleXY;
		FLodSpriteChunk &c = chunks[chunk];
		c.x1 = min(c.x1, inst.pos.X - hw); c.x2 = max(c.x2, inst.pos.X + hw);
		c.y1 = min(c.y1, inst.pos.Y - hw); c.y2 = max(c.y2, inst.pos.Y + hw);
		c.z1 = min(c.z1, inst.pos.Z + min(im.bottom, im.top) * inst.spriteScaleZ);
		c.z2 = max(c.z2, inst.pos.Z + max(im.bottom, im.top) * inst.spriteScaleZ);

		FPlant p;
		const int sec = inst.sector->Index();
		p.secClass = sectorClass[sec];
		p.type = inst.type;
		p.rank = lightRank[sec];
		p.morton = Morton16(uint32_t(cx + 32768), uint32_t(cy + 32768));
		p.chunk = chunk;
		p.sector = sec;
		p.slice = LightSlice(inst.sector, inst.pos);
		p.instance = i;
		plants.Push(p);
	}
	std::sort(plants.begin(), plants.end(), [](const FPlant &a, const FPlant &b)
	{
		if (a.secClass != b.secClass) return a.secClass < b.secClass;
		if (a.type != b.type) return a.type < b.type;
		if (a.rank != b.rank) return a.rank < b.rank;
		if (a.morton != b.morton) return a.morton < b.morton;
		if (a.sector != b.sector) return a.sector < b.sector;
		if (a.slice != b.slice) return a.slice < b.slice;
		return a.instance < b.instance;
	});

	// Runs of plants sharing class, sector class, map square, sector and light slice.
	for (unsigned n = 0; n < plants.Size(); n++)
	{
		const FPlant &p = plants[n];
		const bool same = units.Size() > 0 && units.Last().type == p.type && units.Last().secClass == p.secClass && units.Last().chunk == p.chunk
			&& units.Last().sector == &Level->sectors[p.sector] && n > 0 && plants[n - 1].slice == p.slice;
		if (same) units.Last().numPlants++;
		else
		{
			FLodSpriteUnit u;
			u.firstPlant = n;
			u.numPlants = 1;
			u.chunk = p.chunk;
			u.sector = &Level->sectors[p.sector];
			u.pos = foliage->Instances[p.instance].pos;
			u.type = p.type;
			u.secClass = p.secClass;
			units.Push(u);
		}
	}
	numPlants = plants.Size();
	if (numPlants == 0)
	{
		lodBuildMS = (I_nsTime() - start) / 1e6;
		return;
	}

	if (!planes)
	{
		// Billboards: 4 vertices per plant holding its base, the corner's offset and its yaw.
		TArray<FFlatVertex> verts;
		TArray<uint32_t> indices;
		verts.Resize(plants.Size() * 4);
		indices.Resize(plants.Size() * 6);
		for (unsigned n = 0; n < plants.Size(); n++)
		{
			const FPlant &p = plants[n];
			const FFoliageInstance &inst = foliage->Instances[p.instance];
			const FLodSpriteImage &im = images[typeImage[p.type]];
			const float hw = im.halfWidth * inst.spriteScaleXY;
			const float top = im.top * inst.spriteScaleZ, bottom = im.bottom * inst.spriteScaleZ;
			FFlatVertex *v = &verts[n * 4];
			// corners: top left, top right, bottom right, bottom left (texture v = 0 is the top)
			v[0].Set(inst.pos.X, inst.pos.Z, inst.pos.Y, 0.f, 0.f, -hw, top, inst.yaw);
			v[1].Set(inst.pos.X, inst.pos.Z, inst.pos.Y, 1.f, 0.f, hw, top, inst.yaw);
			v[2].Set(inst.pos.X, inst.pos.Z, inst.pos.Y, 1.f, 1.f, hw, bottom, inst.yaw);
			v[3].Set(inst.pos.X, inst.pos.Z, inst.pos.Y, 0.f, 1.f, -hw, bottom, inst.yaw);
			const uint32_t b = n * 4;
			uint32_t *ix = &indices[n * 6];
			ix[0] = b; ix[1] = b + 1; ix[2] = b + 2;
			ix[3] = b; ix[4] = b + 2; ix[5] = b + 3;
		}
		static const FVertexBufferAttribute format[] = {
			{ 0, VATTR_VERTEX, VFmt_Float3, (int)myoffsetof(FFlatVertex, x) },
			{ 0, VATTR_TEXCOORD, VFmt_Float2, (int)myoffsetof(FFlatVertex, u) },
			{ 0, VATTR_LIGHTMAP, VFmt_Float3, (int)myoffsetof(FFlatVertex, lu) },
		};
		vb = screen->CreateVertexBuffer();
		vb->SetFormat(1, 3, sizeof(FFlatVertex), format);
		vb->SetData(verts.Size() * sizeof(FFlatVertex), verts.Data(), BufferUsageType::Static);
		ib = screen->CreateIndexBuffer();
		ib->SetData(indices.Size() * sizeof(uint32_t), indices.Data(), BufferUsageType::Static);
		lodBuildMS = (I_nsTime() - start) / 1e6;
		return;
	}

	// Planes: one buffer of 48-byte records. First each class's plane mesh (8 one-sided quads on 4 planes, 32
	// vertices, in the class's own space at its own scale, yaw 0), then one record per plant: where that space's
	// origin is and the 3x3 matrix that turns, tilts and scales it like the plant's model. Drawn instanced: binding 0
	// walks the mesh, binding 1 the plants (main.vp).
	struct FPlaneVertex { float x, y, z, u, v, nx, parity, nz, pad[4]; };
	struct FPlaneInstance { float bx, by, bz, c0x, c0y, c0z, c1x, c1y, c1z, c2x, c2y, c2z; };
	static_assert(sizeof(FPlaneVertex) == 48 && sizeof(FPlaneInstance) == 48, "LOD sprite plane records must share their stride");
	TArray<FPlaneVertex> meshes;
	imageMesh.Resize(images.Size());
	for (unsigned i = 0; i < images.Size(); i++)
	{
		imageMesh[i] = meshes.Size();
		const FLodSpriteImage &im = images[i];
		for (int k = 0; k < LODSPRITE_ANGLES; k++)
		{
			// Side k faces the eye at k * 45 degrees from the front and shows frame k (as the sprite was made: its right
			// is (-sin, cos)). Sides k and k + 4 lie on the same plane, back to back.
			const double a = k * (2 * M_PI / LODSPRITE_ANGLES);
			const float nx = (float)cos(a), ny = (float)sin(a);
			const float rx = -ny, ry = nx;
			const float u0 = float(k) / LODSPRITE_ANGLES, u1 = float(k + 1) / LODSPRITE_ANGLES;
			const float corner[4][3] = { { -1.f, im.top, 0.f }, { 1.f, im.top, 1.f }, { 1.f, im.bottom, 1.f }, { -1.f, im.bottom, 0.f } };	// side, height, u
			const float vs[4] = { 0.f, 0.f, 1.f, 1.f };
			for (int c = 0; c < 4; c++)
			{
				FPlaneVertex pv = {};
				const float s = corner[c][0] * im.halfWidth;
				// renderer coordinates: (x, height, y)
				pv.x = rx * s;
				pv.y = corner[c][1];
				pv.z = ry * s;
				pv.u = corner[c][2] > 0.5f ? u1 : u0;
				pv.v = vs[c];
				pv.nx = nx;
				pv.parity = float(k & 1);
				pv.nz = ny;
				meshes.Push(pv);
			}
		}
	}

	// Per foliage type: the inverse of the model's own transform at yaw 0 and class scale, the space the sprite was made in.
	TArray<VSMatrix> typeCanonical, typeCanonicalInv;
	typeCanonical.Resize(foliage->Types.Size());
	typeCanonicalInv.Resize(foliage->Types.Size());
	for (unsigned t = 0; t < foliage->Types.Size(); t++)
	{
		const FFoliageType &ft = foliage->Types[t];
		typeCanonical[t] = ft.smf->ObjectToWorldMatrix(Level, DVector3(0, 0, 0), DRotator(nullAngle, nullAngle, nullAngle), GetDefaultByType(ft.cls)->Scale, ft.smfFlags, 0.);
		if (!typeCanonical[t].inverseMatrix(typeCanonicalInv[t])) typeCanonicalInv[t].loadIdentity();
	}

	TArray<FPlaneInstance> plantRecords;
	plantRecords.Resize(plants.Size());
	for (unsigned n = 0; n < plants.Size(); n++)
	{
		const FPlant &p = plants[n];
		const FFoliageInstance &inst = foliage->Instances[p.instance];
		// world = M * v and sprite space = L0 * v, so world = M * L0^-1 * spriteSpace: the plant's own matrix after the
		// inverse of the canonical one. Column major: m[col * 4 + row].
		VSMatrix rel = inst.matrix;
		rel.multMatrix(typeCanonicalInv[p.type]);
		const float *m = rel.get();
		FPlaneInstance &r = plantRecords[n];
		r.bx = m[12]; r.by = m[13]; r.bz = m[14];
		r.c0x = m[0]; r.c0y = m[1]; r.c0z = m[2];
		r.c1x = m[4]; r.c1y = m[5]; r.c1z = m[6];
		r.c2x = m[8]; r.c2y = m[9]; r.c2z = m[10];
	}

	firstInstance = meshes.Size();
	TArray<uint8_t> data;
	data.Resize((meshes.Size() + plantRecords.Size()) * 48);
	memcpy(data.Data(), meshes.Data(), meshes.Size() * 48);
	memcpy(data.Data() + meshes.Size() * 48, plantRecords.Data(), plantRecords.Size() * 48);

	static const FVertexBufferAttribute planeFormat[] = {
		{ 0, VATTR_VERTEX, VFmt_Float3, 0 },
		{ 0, VATTR_TEXCOORD, VFmt_Float2, 12 },
		{ 0, VATTR_LIGHTMAP, VFmt_Float3, 20 },
		{ 1, VATTR_VERTEX2, VFmt_Float4, 0 },
		{ 1, VATTR_NORMAL2, VFmt_Float4, 16 },
		{ 1, VATTR_BONEWEIGHT, VFmt_Float4, 32 },
	};
	vb = screen->CreateVertexBuffer();
	vb->SetInstancedBindings(2);	// binding 1: one record per plant
	vb->SetFormat(2, 6, 48, planeFormat);
	vb->SetData(data.Size(), data.Data(), BufferUsageType::Static);

	uint32_t quadIndices[LODSPRITE_ANGLES * 6];
	for (int k = 0; k < LODSPRITE_ANGLES; k++)
	{
		const uint32_t b = k * 4;
		uint32_t *ix = &quadIndices[k * 6];
		ix[0] = b; ix[1] = b + 1; ix[2] = b + 2;
		ix[3] = b; ix[4] = b + 2; ix[5] = b + 3;
	}
	ib = screen->CreateIndexBuffer();
	ib->SetData(sizeof(quadIndices), quadIndices, BufferUsageType::Static);
	lodBuildMS = (I_nsTime() - start) / 1e6;
}

//==========================================================================
//
// Before each frame (main thread, before the BSP walk)
//
//==========================================================================

static bool LodSpritesSupported()
{
	return gl_lodsprites && screen->Backend() != 0;	// GLES has no vertex path for them
}

void UpdateLodSprites(FLevelLocals *Level)
{
	if (!LodSpritesSupported() || Level->sectors.Size() == 0)
	{
		if (Level->LodSprites)
		{
			DeleteLodSprites(Level->LodSprites);
			Level->LodSprites = nullptr;
		}
		return;
	}
	FLodSprites *ls = Level->LodSprites;
	if (ls == nullptr) ls = Level->LodSprites = new FLodSprites;
	if (!ls->Matches(Level)) ls->Reset(Level);

	// Make the sprites of every LODSPRITE actor class in the level now (world load), not when one first comes into view.
	if (!ls->actorsScanned)
	{
		ls->actorsScanned = true;
		for (AActor *mo : Level->LodSpriteActors)
		{
			if (!(mo->perfFlags & PERF_CLUSTERPROXY)) ls->ImageFor(Level, mo->GetClass());
		}
	}

	FFoliageSystem *foliage = Level->Foliage;
	const unsigned instances = foliage ? foliage->Instances.Size() : 0;
	if (foliage != ls->builtFoliage || instances != ls->builtInstances) ls->BuildBatch(Level);	// new plants: their classes are made here too
}

bool LodSpriteTypeInBatch(FLevelLocals *Level, unsigned foliageType)
{
	FLodSprites *ls = Level->LodSprites;
	return ls && foliageType < ls->typeImage.Size() && ls->typeImage[foliageType] >= 0 && ls->vb != nullptr;
}

bool LodFarModelType(FLevelLocals *Level, unsigned foliageType)
{
	FLodSprites *ls = Level->LodSprites;
	return ls && foliageType < ls->typeFarModel.Size() && ls->typeFarModel[foliageType];
}

const TArray<uint32_t> *LodFarModelPlants(FLevelLocals *Level)
{
	FLodSprites *ls = Level->LodSprites;
	return ls ? &ls->farPlants : nullptr;
}

// LODPLANE plants: their model is drawn by DrawFarFoliage from 'start' on (the line cull distance; 0 in lodonly sectors
// the walk skips) up to 'outer' (their "None" distance or DistanceCheck; past the cull distance only where the world
// LOD shows the level). Nearer than 'start' the normal foliage path draws them.
void HWDrawInfo::LodFarModelWindow(PClassActor *cls, int sectorClass, double &start, double &outer) const
{
	double hideDist = LODSPRITE_NEVER;
	if (lodSpriteScale > 0)
	{
		for (const LODmodel *m = cls->ActorInfo()->LODmodels; m != nullptr; m = m->Next)
		{
			if (m->type == LOD_Hide) { hideDist = m->distance / lodSpriteScale; break; }
		}
	}
	FIntCVar *dc = cls->ActorInfo()->distancecheck;
	if (dc != nullptr && **dc >= 0) hideDist = min(hideDist, double(**dc));
	outer = hideDist;
	if ((sectorClass & LODSPRITE_SEC_LODONLY) && lodOnly)
	{
		start = 0;
		return;
	}
	start = lodSpriteCull > 0 ? lodSpriteCull : LODSPRITE_NEVER;	// no line cull: the walk reaches everything
	if (!((sectorClass & LODSPRITE_SEC_MODEL) && worldLOD != nullptr)) outer = min(outer, start);
}

const uint8_t *LodSpriteSectorClasses(FLevelLocals *Level)
{
	FLodSprites *ls = Level->LodSprites;
	return ls ? ls->sectorClass.Data() : nullptr;
}

//==========================================================================
//
// Per view
//
//==========================================================================

void HWDrawInfo::SetupLodSprites()
{
	lodSprites = false;
	FLodSprites *ls = Level->LodSprites;
	if (ls == nullptr || !LodSpritesSupported() || !ls->Matches(Level) || outer != nullptr || mCurrentPortal != nullptr || Viewpoint.bDoOrtho) return;
	lodSprites = true;
	lodSpriteCull = (r_distance_cull_type > 0 && Viewpoint.culldistsq > 0) ? sqrt(Viewpoint.culldistsq) : 0.;
	lodSpriteScale = clamp<double>(r_lod, 0., 1000.);
}

// Where a LODSPRITE class is a sprite in this view: from its LODModel "Sprite" distance or the line cull distance,
// whichever is nearer, up to its "None" distance (or DistanceCheck). Past the line cull distance only where the world
// LOD shows the level. In lodonly sectors that the BSP walk skips, at any distance.
//
// Cross-fade (gl_lodsprite_fade): pixels between fadeFrom and fadeTo from the eye are dithered between the two. Pixels
// of a thing can be up to its radius r nearer or farther than its origin, so the band ends r before the model's end
// (all of the model's pixels are faded out by then) and the sprite starts r before the band (none of its pixels show yet).
void HWDrawInfo::LodSpriteWindow(PClassActor *cls, int sectorClass, FLodSpriteWindow &w) const
{
	double inner, outer;
	double spriteDist = LODSPRITE_NEVER, hideDist = LODSPRITE_NEVER;
	if (lodSpriteScale > 0)
	{
		for (const LODmodel *m = cls->ActorInfo()->LODmodels; m != nullptr; m = m->Next)
		{
			if (m->type == LOD_Sprite && spriteDist >= LODSPRITE_NEVER) spriteDist = m->distance / lodSpriteScale;
			else if (m->type == LOD_Hide && hideDist >= LODSPRITE_NEVER) hideDist = m->distance / lodSpriteScale;
		}
	}
	FIntCVar *dc = cls->ActorInfo()->distancecheck;
	if (dc != nullptr && **dc >= 0) hideDist = min(hideDist, double(**dc));

	const double cull = lodSpriteCull > 0 ? lodSpriteCull : LODSPRITE_NEVER;
	if ((sectorClass & LODSPRITE_SEC_LODONLY) && lodOnly)
	{
		inner = 0;
		outer = hideDist;
	}
	else
	{
		inner = min(spriteDist, cull);
		outer = hideDist;
		if (!((sectorClass & LODSPRITE_SEC_MODEL) && worldLOD != nullptr)) outer = min(outer, cull);
	}
	w.modelEnd = w.spriteStart = inner;
	w.outer = outer;
	w.fadeFrom = w.fadeTo = 0;

	const double fade = gl_lodsprite_fade;
	FLodSprites *ls = Level->LodSprites;
	if (fade > 0 && inner > 0 && inner < LODSPRITE_NEVER && ls != nullptr)
	{
		const FLodSpriteImage *img = ls->Find(cls);
		if (img != nullptr && img->valid)
		{
			const double scale = max(1., (double)cls->ActorInfo()->FoliageScaleMax);
			const double r = sqrt(double(img->halfWidth) * img->halfWidth + max(double(img->top) * img->top, double(img->bottom) * img->bottom)) * scale;
			const double to = inner - r, from = to - fade;
			if (from > r)
			{
				w.fadeFrom = (float)from;
				w.fadeTo = (float)to;
				w.spriteStart = from - r;
			}
		}
	}
}

bool HWDrawInfo::LodSpriteRange(PClassActor *cls, const sector_t *sec, FLodSpriteWindow &w) const
{
	if (!lodSprites) return false;
	FLodSprites *ls = Level->LodSprites;
	const FLodSpriteImage *img = ls->Find(cls);
	if (img == nullptr || !img->valid) return false;
	LodSpriteWindow(cls, sec ? ls->sectorClass[sec->Index()] : 0, w);
	return true;
}

//==========================================================================
//
// Drawing (opaque pass, after the models and foliage)
//
//==========================================================================

struct FActorSprite
{
	int image;
	int light;
	float fadeFrom, fadeTo;
	AActor *actor;
	DVector3 pos;
};
static TArray<FActorSprite> actorSprites;
static TArray<double> windowInnerSq, windowOuter;	// per (foliage type, sector class): squared sprite start (-1: never drawn), outer
static TArray<FVector2> windowFade;					// per (foliage type, sector class): cross-fade band

static inline float ShaderOuterSq(double outer)
{
	return outer >= LODSPRITE_NEVER ? 0.f : (float)(outer * outer);
}

void HWDrawInfo::DrawLodSprites(FRenderState &state)
{
	FLodSprites *ls = Level->LodSprites;
	if (!lodSprites || ls == nullptr || !gl_render_things) return;
	lodStats.time.Clock();

	const DVector3 eye = Viewpoint.bDoOob ? Viewpoint.OffPos : Viewpoint.Pos;
	const FVector3 eyeF((float)eye.X, (float)eye.Y, (float)eye.Z);
	const float viewCos = (float)Viewpoint.ViewVector.X, viewSin = (float)Viewpoint.ViewVector.Y;
	const float mipAlpha = max(0.f, *gl_lodsprite_mipalpha);
	bool fadeSet = false;
	auto setFade = [&](float from, float to)
	{
		if (to > from)
		{
			state.SetLodFade(eyeF, true, from, to);	// dithered in as the model goes
			fadeSet = true;
		}
		else if (fadeSet)
		{
			state.ClearLodFade();
			fadeSet = false;
		}
	};
	ResetFoliageLightGroups(Level);

	state.EnableModelMatrix(false);
	state.EnableTextureMatrix(false);
	state.SetInterpolationFactor(0.f);
	state.SetBoneIndexBase(-1);
	state.SetInstanceBase(-1);
	state.SetDynLight(0, 0, 0);
	state.SetLightIndex(-1);
	state.SetRenderStyle(DefaultRenderStyle());
	state.SetTextureMode(TM_NORMAL);
	state.EnableTexture(true);
	state.SetCulling(Cull_None);
	state.SetDepthFunc(DF_Less);
	state.AlphaFunc(Alpha_GEqual, clamp<float>(gl_lodsprite_alpha, 0.f, 1.f));

	int curImage = -1, curLight = -1;
	auto setImage = [&](int image)
	{
		if (image == curImage) return;
		curImage = image;
		state.SetMaterial(ls->images[image].tex, UF_Skin, 0, CLAMP_XY, NO_TRANSLATION, -1, nullptr);
	};
	auto setLight = [&](int light)
	{
		if (light == curLight) return;
		curLight = light;
		ApplyFoliageLightGroup(this, state, light);
	};

	// Cluster plants: static batch, the shader picks per plant.
	FFoliageSystem *foliage = Level->Foliage;
	if (ls->vb != nullptr && foliage != nullptr && gl_render_foliage)
	{
		const unsigned numTypes = ls->typeImage.Size();
		windowInnerSq.Resize(numTypes * 4);
		windowOuter.Resize(numTypes * 4);
		windowFade.Resize(numTypes * 4);
		for (unsigned t = 0; t < numTypes; t++)
		{
			for (int c = 0; c < 4; c++)
			{
				FLodSpriteWindow w = { LODSPRITE_NEVER, LODSPRITE_NEVER, 0, 0, 0 };
				if (ls->typeImage[t] >= 0) LodSpriteWindow(foliage->Types[t].cls, c, w);
				const bool empty = w.spriteStart >= LODSPRITE_NEVER || w.outer <= w.spriteStart;
				windowInnerSq[t * 4 + c] = empty ? -1. : w.spriteStart * w.spriteStart;
				windowOuter[t * 4 + c] = w.outer;
				windowFade[t * 4 + c] = FVector2(w.fadeFrom, w.fadeTo);
			}
		}

		const bool frustum = Frustum.valid;
		for (auto &c : ls->chunks)
		{
			c.visible = !frustum || Frustum.MapBoxVisible(c.x1, c.x2, c.y1, c.y2, c.z1, c.z2);
			if (!c.visible) continue;
			const double nx = eye.X < c.x1 ? c.x1 - eye.X : eye.X > c.x2 ? eye.X - c.x2 : 0.;
			const double ny = eye.Y < c.y1 ? c.y1 - eye.Y : eye.Y > c.y2 ? eye.Y - c.y2 : 0.;
			const double fx = max(fabs(c.x1 - eye.X), fabs(c.x2 - eye.X)), fy = max(fabs(c.y1 - eye.Y), fabs(c.y2 - eye.Y));
			const double fz = max(fabs(c.z1 - eye.Z), fabs(c.z2 - eye.Z));
			c.nearSq = nx * nx + ny * ny;
			c.farSq = fx * fx + fy * fy + fz * fz;
			lodStats.chunks++;
		}

		const bool planes = ls->planes;
		if (!planes) state.SetVertexBuffer(ls->vb, 0, 0);
		state.SetIndexBuffer(ls->ib);
		int runType = -1, runClass = -1, runLight = -1;
		unsigned runFirst = 0, runCount = 0;	// plants
		auto flush = [&]()
		{
			if (runCount == 0) return;
			const unsigned w = runType * 4 + runClass;
			const int image = ls->typeImage[runType];
			setImage(image);
			setLight(runLight);
			if (planes) state.SetSpriteLod(eyeF, (float)windowInnerSq[w], ShaderOuterSq(windowOuter[w]), (float)(LODSPRITE_PLANE_ANGLE * (M_PI / 180.)), 0.f, mipAlpha, true);
			else state.SetSpriteLod(eyeF, (float)windowInnerSq[w], ShaderOuterSq(windowOuter[w]), viewCos, viewSin, mipAlpha);
			setFade(windowFade[w].X, windowFade[w].Y);
			if (planes)
			{
				// the class's plane mesh, once per plant of the run
				state.SetVertexBuffer(ls->vb, ls->imageMesh[image], ls->firstInstance + runFirst);
				state.DrawIndexedInstanced(DT_Triangles, 0, LODSPRITE_ANGLES * 6, runCount);
			}
			else state.DrawIndexed(DT_Triangles, runFirst * 6, runCount * 6);
			lodStats.draws++;
			runCount = 0;
		};
		for (auto &u : ls->units)
		{
			const FLodSpriteChunk &c = ls->chunks[u.chunk];
			const unsigned w = u.type * 4 + u.secClass;
			const double innerSq = windowInnerSq[w], outer = windowOuter[w];
			if (!c.visible || innerSq < 0 || c.farSq < innerSq || (outer < LODSPRITE_NEVER && c.nearSq >= outer * outer))
			{
				flush();
				continue;
			}
			const int light = FoliageLightGroup(this, u.sector, u.pos);
			if (runCount > 0 && (u.type != runType || u.secClass != runClass || light != runLight || u.firstPlant != runFirst + runCount)) flush();
			if (runCount == 0)
			{
				runType = u.type;
				runClass = u.secClass;
				runLight = light;
				runFirst = u.firstPlant;
			}
			runCount += u.numPlants;
			lodStats.units++;
		}
		flush();
	}

	// Actors: each frame's quads in the streamed vertex buffer. Same tests as HWSprite::Process, in reverse.
	actorSprites.Clear();
	AActor *camera = Viewpoint.camera;
	for (AActor *mo : Level->LodSpriteActors)
	{
		if ((mo->perfFlags & (PERF_LODSPRITE | PERF_CLUSTERPROXY)) != PERF_LODSPRITE) continue;
		if (mo->sprite == 0 || mo->picnum.isValid() || !mo->IsVisibleToPlayer()) continue;
		if ((mo->renderflags & RF_INVISIBLE) || !mo->RenderStyle.IsVisible(mo->InterpolatedAlpha(Viewpoint.TicFrac))) continue;
		if ((mo->renderflags2 & RF2_ONLYVISIBLEINMIRRORS) || (mo == camera && !Viewpoint.showviewer)) continue;
		if ((!r_debug_disable_vis_filter && !!(mo->RenderRequired & ~r_renderercaps)) || !!(mo->RenderHidden & r_renderercaps)) continue;
		if (FindModelFrame(mo, mo->sprite, mo->frame, !!(mo->flags & MF_DROPPED)) == nullptr) continue;	// not a model now: drawn as usual

		PClassActor *cls = mo->GetClass();
		int *index = ls->classIndex.CheckKey(cls);
		if (index == nullptr)
		{
			// New class: drawn as a model this frame (HWSprite::Process found no sprite either), made after this frame's sprites.
			if (ls->pending.Find(cls) == ls->pending.Size()) ls->pending.Push(cls);
			continue;
		}
		if (!ls->images[*index].valid) continue;
		FLodSpriteWindow w;
		LodSpriteWindow(cls, ls->sectorClass[mo->Sector->Index()], w);
		const DVector3 pos = mo->InterpolatedPosition(Viewpoint.TicFrac);
		const double distSq = (pos - (Viewpoint.bDoOob ? Viewpoint.OffPos : Viewpoint.Pos)).LengthSquared();
		if (w.spriteStart >= LODSPRITE_NEVER || distSq < w.spriteStart * w.spriteStart || (w.outer < LODSPRITE_NEVER && distSq >= w.outer * w.outer)) continue;
		if (Frustum.valid)
		{
			const FLodSpriteImage &im = ls->images[*index];
			const double r = im.halfWidth * fabs(mo->Scale.X) + 1;
			if (!Frustum.MapBoxVisible(pos.X - r, pos.X + r, pos.Y - r, pos.Y + r, pos.Z + im.bottom * fabs(mo->Scale.Y) - 1, pos.Z + im.top * fabs(mo->Scale.Y) + 1)) continue;
		}
		FActorSprite as;
		as.image = *index;
		as.fadeFrom = w.fadeFrom;
		as.fadeTo = w.fadeTo;
		as.actor = mo;
		as.pos = pos + mo->WorldOffset;
		as.light = FoliageLightGroup(this, mo->Sector, FVector3((float)pos.X, (float)pos.Y, (float)pos.Z));
		actorSprites.Push(as);
	}
	if (actorSprites.Size() > 0)
	{
		std::sort(actorSprites.begin(), actorSprites.end(), [](const FActorSprite &a, const FActorSprite &b)
		{
			if (a.image != b.image) return a.image < b.image;
			if (a.fadeTo != b.fadeTo) return a.fadeTo < b.fadeTo;
			return a.light < b.light;
		});
		auto alloc = screen->mVertexData->AllocVertices(actorSprites.Size() * 6);
		FFlatVertex *v = alloc.first;
		for (auto &as : actorSprites)
		{
			AActor *mo = as.actor;
			const FLodSpriteImage &im = ls->images[as.image];
			const AActor *def = GetDefaultByType(mo->GetClass());
			const float sxy = def->Scale.X != 0 ? (float)fabs(mo->Scale.X / def->Scale.X) : 1.f;
			const float sz = def->Scale.Y != 0 ? (float)fabs(mo->Scale.Y / def->Scale.Y) : 1.f;
			const float hw = im.halfWidth * sxy, top = im.top * sz, bottom = im.bottom * sz;
			const float yaw = (float)(mo->Angles.Yaw + mo->SpriteRotation).Radians();
			const float x = (float)as.pos.X, y = (float)as.pos.Y, z = (float)as.pos.Z;
			v[0].Set(x, z, y, 0.f, 0.f, -hw, top, yaw);
			v[1].Set(x, z, y, 1.f, 0.f, hw, top, yaw);
			v[2].Set(x, z, y, 1.f, 1.f, hw, bottom, yaw);
			v[3] = v[0];
			v[4] = v[2];
			v[5].Set(x, z, y, 0.f, 1.f, -hw, bottom, yaw);
			v += 6;
		}
		state.SetVertexBuffer(screen->mVertexData);
		state.SetSpriteLod(eyeF, 0.f, 0.f, viewCos, viewSin, mipAlpha);	// the distance test was made above
		unsigned i = 0;
		while (i < actorSprites.Size())
		{
			unsigned j = i + 1;
			while (j < actorSprites.Size() && actorSprites[j].image == actorSprites[i].image && actorSprites[j].light == actorSprites[i].light
				&& actorSprites[j].fadeFrom == actorSprites[i].fadeFrom && actorSprites[j].fadeTo == actorSprites[i].fadeTo) j++;
			setImage(actorSprites[i].image);
			setLight(actorSprites[i].light);
			setFade(actorSprites[i].fadeFrom, actorSprites[i].fadeTo);
			state.Draw(DT_Triangles, alloc.second + i * 6, (j - i) * 6);
			lodStats.actorDraws++;
			i = j;
		}
		lodStats.actors += actorSprites.Size();
	}

	// Leave the state as the model drawer does.
	state.ClearSpriteLod();
	state.ClearLodFade();
	state.SetObjectColor(0xffffffff);
	state.SetAddColor(0);
	state.SetDynLight(0, 0, 0);
	state.SetLightIndex(-1);
	state.EnableTexture(true);
	state.AlphaFunc(Alpha_GEqual, gl_mask_threshold);
	state.SetVertexBuffer(screen->mVertexData);

	// Classes met for the first time this frame: make them now, so they're drawn as sprites from the next frame on.
	if (ls->pending.Size() > 0)
	{
		for (auto cls : ls->pending) ls->ImageFor(Level, cls);
		ls->pending.Clear();
	}
	lodStats.time.Unclock();
}

//==========================================================================
//
// Diagnostics
//
//==========================================================================

ADD_STAT(lodsprites)
{
	FString out;
	FLodSprites *ls = nullptr;
	for (auto Level : AllLevels()) if (Level->LodSprites) { ls = Level->LodSprites; break; }
	if (ls == nullptr)
	{
		out = !gl_lodsprites ? "LOD sprites: off (gl_lodsprites 0)" : screen->Backend() == 0 ? "LOD sprites: not supported on GLES" : "LOD sprites: none";
		return out;
	}
	int valid = 0;
	for (auto &im : ls->images) valid += im.valid;
	out.Format("LOD sprites: %d classes (%d with a sprite; %d made this game, last in %.1f ms) | foliage (%s): %u plants, %u runs, %u squares, built in %.1f ms\n"
		"This frame: %d squares in view, %d runs in %d draws, %d actors in %d draws, %.3f ms",
		ls->images.Size(), valid, lodGenCount, lodGenMS, ls->planes ? "planes" : "billboards", ls->numPlants, ls->units.Size(), ls->chunks.Size(), lodBuildMS,
		lodStats.chunks, lodStats.units, lodStats.draws, lodStats.actors, lodStats.actorDraws, lodStats.time.TimeMS());
	return out;
}

CCMD(lodsprite_regenerate)
{
	lodImageGeneration++;
	lodImageCache.Clear();
	Printf("LOD sprites will be made again on the next frame.\n");
}

CCMD(lodsprite_list)
{
	FLevelLocals *Level = primaryLevel;
	FLodSprites *ls = Level ? Level->LodSprites : nullptr;
	if (ls == nullptr)
	{
		Printf("No LOD sprites in this level.\n");
		return;
	}
	TMap<PClassActor *, int>::Iterator it(ls->classIndex);
	TMap<PClassActor *, int>::Pair *pair;
	while (it.NextPair(pair))
	{
		const FLodSpriteImage &im = ls->images[pair->Value];
		if (!im.valid)
		{
			Printf(TEXTCOLOR_ORANGE "  %s: none (%s)\n", pair->Key->TypeName.GetChars(), im.error.GetChars());
			continue;
		}
		Printf("  %s: %s, 8 frames of %dx%d, %d colours, from %d triangles, %.0f wide, %.0f to %.0f high\n", pair->Key->TypeName.GetChars(),
			im.tex->GetName().GetChars(), im.frameWidth, im.frameHeight, im.colors, im.triangles, im.halfWidth * 2, im.bottom, im.top);
	}
}
