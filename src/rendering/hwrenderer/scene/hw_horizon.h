/*
** hw_horizon.h
**
** Occlusion horizon for the BSP walk: hides parts of the map that are
** behind hills and under low ceilings, which the angle-only clipper can't.
**
** Below every sector's floor (and above every non-sky ceiling) is solid.
** So a line of sight that crosses the boundary of a subsector the walk has
** already reached must cross it above that subsector's floor and below its
** ceiling. Per slice of yaw, this keeps the lowest slope (dz / distance) a
** line of sight must still have, and the highest slope it may have. The BSP
** walk goes front to back, so everything recorded is in front of what is
** tested afterwards.
**
** Yaw is measured as a 'pseudo angle' (0..4 around the circle, monotonic
** in the real angle), and the direction of every slice boundary comes from
** a table, so there is no trigonometry per edge.
**
**---------------------------------------------------------------------------
**
** Copyright 2026 UZDoom Maintainers and Contributors
**
** SPDX-License-Identifier: GPL-3.0-or-later
**
**---------------------------------------------------------------------------
*/

#pragma once

#include <math.h>
#include <float.h>

struct HWHorizon
{
	enum { NumBins = 1024, ChunkBins = 8 };	// about 0.35 degrees per slice
	static constexpr double BinsPerUnit = NumBins / 4.0;

	float floorSlope[NumBins];	// a line of sight in this slice must be steeper than this to see past the walked ground
	float ceilSlope[NumBins];	// ... and shallower than this to stay under the walked ceilings
	double ex = 0, ey = 0, ez = 0;	// eye

	// unit direction of the start of every slice
	struct Dir { double x, y; };
	Dir dirs[NumBins];
	bool tableReady = false;

	// 0 = +x, 1 = +y, 2 = -x, 3 = -y; monotonic in the real angle
	static double Pseudo(double dx, double dy)
	{
		if (dy >= 0) return dx >= 0 ? dy / (dx + dy) : 1. + (-dx) / (dy - dx);
		return dx < 0 ? 2. + (-dy) / (-dx - dy) : 3. + dx / (dx - dy);
	}

	static Dir PseudoDir(double q)
	{
		const int quadrant = (int)q & 3;
		const double t = q - floor(q);
		double x, y;
		switch (quadrant)
		{
		default:
		case 0: x = 1 - t; y = t; break;
		case 1: x = -t; y = 1 - t; break;
		case 2: x = -(1 - t); y = -t; break;
		case 3: x = t; y = -(1 - t); break;
		}
		const double len = sqrt(x * x + y * y);
		return { x / len, y / len };
	}

	void Clear(double x, double y, double z)
	{
		if (!tableReady)
		{
			for (int i = 0; i < NumBins; i++) dirs[i] = PseudoDir(i / BinsPerUnit);
			tableReady = true;
		}
		ex = x; ey = y; ez = z;
		for (int i = 0; i < NumBins; i++)
		{
			floorSlope[i] = -FLT_MAX;
			ceilSlope[i] = FLT_MAX;
		}
	}

	static double Wrap4(double q)	// to [0, 4)
	{
		q = fmod(q, 4.);
		return q < 0 ? q + 4. : q;
	}

	// One boundary edge of a walked subsector. floorZ is the lowest floor height along it, ceilZ the
	// highest ceiling height; pass hasFloor / hasCeiling false when that plane isn't solid (sky, portal).
	void AddEdge(double x1, double y1, double x2, double y2, double floorZ, double ceilZ, bool hasFloor, bool hasCeiling)
	{
		if (!hasFloor && !hasCeiling) return;
		const double p1x = x1 - ex, p1y = y1 - ey, p2x = x2 - ex, p2y = y2 - ey;
		const double cross = p1x * p2y - p1y * p2x;
		const double dx = p2x - p1x, dy = p2y - p1y;
		const double len2 = dx * dx + dy * dy;
		if (len2 < 1e-6) return;
		// foot of the perpendicular from the eye to the edge's line
		const double t = -(p1x * dx + p1y * dy) / len2;
		const double fx = p1x + t * dx, fy = p1y + t * dy;
		const double perp = sqrt(fx * fx + fy * fy);
		if (perp < 0.5) return;	// eye (nearly) on the line: the edge covers no angle
		const double nx = fx / perp, ny = fy / perp;

		const double q1 = Pseudo(p1x, p1y), q2 = Pseudo(p2x, p2y);
		double start, span;
		if (cross > 0) { start = q1; span = Wrap4(q2 - q1); }
		else { start = q2; span = Wrap4(q1 - q2); }
		if (span <= 0 || span >= 2.) return;

		// only slices the edge covers completely: every line of sight in them crosses the edge
		const long first = (long)ceil(start * BinsPerUnit);
		const long last = (long)floor((start + span) * BinsPerUnit) - 1;
		if (last < first) return;
		const long footBin = (long)floor(Pseudo(nx, ny) * BinsPerUnit);
		const double fz = floorZ - ez, cz = ceilZ - ez;

		for (long k0 = first; k0 <= last; k0 += ChunkBins)
		{
			const long k1 = k0 + ChunkBins - 1 < last ? k0 + ChunkBins - 1 : last;
			// distance along the chunk's two boundary directions to the line
			const Dir &ua = dirs[k0 % NumBins], &ub = dirs[(k1 + 1) % NumBins];
			const double ca = ua.x * nx + ua.y * ny, cb = ub.x * nx + ub.y * ny;
			if (ca <= 1e-9 || cb <= 1e-9) continue;
			const double da = perp / ca, db = perp / cb;
			// distance is smallest at the foot of the perpendicular, if it's in the chunk
			const long kf = ((footBin - k0) % NumBins + NumBins) % NumBins;
			const bool footIn = kf <= k1 - k0;
			const double dmin = footIn ? perp : (da < db ? da : db);
			const double dmax = da > db ? da : db;

			float fs = -FLT_MAX, cs = FLT_MAX;
			if (hasFloor) fs = (float)(fz < 0 ? fz / dmin : fz / dmax);	// least steep requirement along the chunk
			if (hasCeiling) cs = (float)(cz > 0 ? cz / dmin : cz / dmax);	// most generous limit along the chunk
			for (long k = k0; k <= k1; k++)
			{
				const int b = (int)(k % NumBins);
				if (fs > floorSlope[b]) floorSlope[b] = fs;
				if (cs < ceilSlope[b]) ceilSlope[b] = cs;
			}
		}
	}

	// Is every line of sight to this box (map coordinates) blocked by what was added so far?
	bool BoxHidden(double x1, double x2, double y1, double y2, double z1, double z2) const
	{
		if (x1 > x2 || y1 > y2) return false;
		const double bx1 = x1 - ex, bx2 = x2 - ex, by1 = y1 - ey, by2 = y2 - ey;
		// distance from the eye to the box
		const double nx = bx1 > 0 ? bx1 : (bx2 < 0 ? bx2 : 0);
		const double ny = by1 > 0 ? by1 : (by2 < 0 ? by2 : 0);
		const double dmin = sqrt(nx * nx + ny * ny);
		if (dmin < 1.) return false;
		const double fx = fabs(bx1) > fabs(bx2) ? bx1 : bx2, fy = fabs(by1) > fabs(by2) ? by1 : by2;
		const double dmax = sqrt(fx * fx + fy * fy);

		// yaw range: corners relative to the direction of the box centre
		const double ref = Pseudo((bx1 + bx2) * 0.5, (by1 + by2) * 0.5);
		double lo = 0, hi = 0;
		const double cx[4] = { bx1, bx2, bx1, bx2 }, cy[4] = { by1, by1, by2, by2 };
		for (int i = 0; i < 4; i++)
		{
			double d = Pseudo(cx[i], cy[i]) - ref;
			if (d > 2.) d -= 4.;
			else if (d < -2.) d += 4.;
			if (d < lo) lo = d;
			if (d > hi) hi = d;
		}
		if (hi - lo >= 2.) return false;

		const double tz = z2 - ez, bz = z1 - ez;
		const double topSlope = tz > 0 ? tz / dmin : tz / dmax;		// steepest line of sight that reaches the box
		const double bottomSlope = bz < 0 ? bz / dmin : bz / dmax;	// shallowest
		const double eps = 1e-5;

		const double base = Wrap4(ref + lo);
		const long first = (long)floor(base * BinsPerUnit);
		const long last = (long)floor((base + (hi - lo)) * BinsPerUnit);
		for (long k = first; k <= last; k++)
		{
			const int b = (int)(k % NumBins);
			const double f = floorSlope[b], c = ceilSlope[b];
			if (topSlope < f - eps) continue;		// all below the ground in front
			if (bottomSlope > c + eps) continue;	// all above the ceilings in front
			if (f > c + eps) continue;				// nothing gets through this slice at all
			return false;
		}
		return true;
	}
};
