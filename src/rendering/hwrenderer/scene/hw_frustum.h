/*
** hw_frustum.h
**
** The view frustum of one HWDrawInfo, taken from its projection and view
** matrices, for culling things the angular clipper can't (anything behind
** the camera once the view pitches far enough to see straight down) and
** for measuring how much of the yaw circle the view can actually see.
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
#include "matrix.h"
#include "vectors.h"

// Planes are in the renderer's world convention, which is what the view matrix
// and every object-to-world matrix use: (map X, map Z, map Y). A point is on the
// inside of a plane when a*x + b*y + c*z + d >= 0. There is no far plane: the
// renderer's far distance is effectively unlimited.
struct HWFrustum
{
	struct Plane { double a, b, c, d; };
	enum { Left, Right, Bottom, Top, Near, NumPlanes };

	Plane planes[NumPlanes];
	bool valid = false;

	void Init(const VSMatrix &projection, const VSMatrix &view)
	{
		VSMatrix m = projection;
		m.multMatrix(view);
		const FLOATTYPE *e = m.get();	// column major: element (row r, column c) is e[c*4 + r]
		auto row = [&](int r, int i) { return (double)e[i * 4 + r]; };
		auto set = [&](int index, int r, double sign)
		{
			// Gribb/Hartmann: row 3 +/- row r. For the near plane this is GL's -w <= z, which is
			// also safe (just looser) for a 0..w depth range.
			Plane &p = planes[index];
			p.a = row(3, 0) + sign * row(r, 0);
			p.b = row(3, 1) + sign * row(r, 1);
			p.c = row(3, 2) + sign * row(r, 2);
			p.d = row(3, 3) + sign * row(r, 3);
			const double len = sqrt(p.a * p.a + p.b * p.b + p.c * p.c);
			if (len < 1e-12) return false;
			p.a /= len; p.b /= len; p.c /= len; p.d /= len;
			return true;
		};
		valid = set(Left, 0, 1) && set(Right, 0, -1) && set(Bottom, 1, 1) && set(Top, 1, -1) && set(Near, 2, 1);
	}

	// Sphere in map coordinates.
	bool SphereVisible(double x, double y, double z, double radius) const
	{
		if (!valid) return true;
		for (const Plane &p : planes)
		{
			if (p.a * x + p.b * z + p.c * y + p.d < -radius) return false;
		}
		return true;
	}

	// Axis-aligned box in map coordinates.
	bool MapBoxVisible(double x1, double x2, double y1, double y2, double z1, double z2) const
	{
		if (!valid) return true;
		// renderer convention: (map X, map Z, map Y)
		const double c[3] = { (x1 + x2) * 0.5, (z1 + z2) * 0.5, (y1 + y2) * 0.5 };
		const double h[3] = { fabs(x2 - x1) * 0.5, fabs(z2 - z1) * 0.5, fabs(y2 - y1) * 0.5 };
		for (const Plane &p : planes)
		{
			const double dist = p.a * c[0] + p.b * c[1] + p.c * c[2] + p.d;
			const double reach = fabs(p.a) * h[0] + fabs(p.b) * h[1] + fabs(p.c) * h[2];
			if (dist < -reach) return false;
		}
		return true;
	}

	// Box given in an object's own coordinates, placed in the world by an object-to-world
	// matrix (renderer convention, as built by FSpriteModelFrame::ObjectToWorldMatrix).
	bool ObjectBoxVisible(const VSMatrix &objectToWorld, const FVector3 &mins, const FVector3 &maxs, double padding) const
	{
		if (!valid) return true;
		const FLOATTYPE *e = objectToWorld.get();
		const double c[3] = { (mins.X + maxs.X) * 0.5, (mins.Y + maxs.Y) * 0.5, (mins.Z + maxs.Z) * 0.5 };
		const double h[3] = { (maxs.X - mins.X) * 0.5, (maxs.Y - mins.Y) * 0.5, (maxs.Z - mins.Z) * 0.5 };
		double wc[3], we[3];
		for (int i = 0; i < 3; i++)
		{
			wc[i] = e[i] * c[0] + e[4 + i] * c[1] + e[8 + i] * c[2] + e[12 + i];
			we[i] = fabs(e[i]) * h[0] + fabs(e[4 + i]) * h[1] + fabs(e[8 + i]) * h[2] + padding;
		}
		for (const Plane &p : planes)
		{
			const double dist = p.a * wc[0] + p.b * wc[1] + p.c * wc[2] + p.d;
			const double reach = fabs(p.a) * we[0] + fabs(p.b) * we[1] + fabs(p.c) * we[2];
			if (dist < -reach) return false;
		}
		return true;
	}

	// The largest yaw difference, in degrees, between viewYaw and any direction inside the frustum.
	// Returns a negative value when the frustum contains a vertical ray (looking far enough up or
	// down), because then every yaw is visible.
	double MaxYawDeviation(double viewYawRadians) const
	{
		if (!valid) return -1;
		static const int corners[4][2] = { { Left, Bottom }, { Left, Top }, { Right, Bottom }, { Right, Top } };
		const double fx = cos(viewYawRadians), fy = sin(viewYawRadians);
		const Plane &n = planes[Near];
		double maxang = 0;
		for (auto &cn : corners)
		{
			// Each edge of the frustum is where two neighbouring side planes meet.
			const Plane &p = planes[cn[0]], &q = planes[cn[1]];
			double dx = p.b * q.c - p.c * q.b;
			double dy = p.c * q.a - p.a * q.c;
			double dz = p.a * q.b - p.b * q.a;
			if (dx * n.a + dy * n.b + dz * n.c < 0) { dx = -dx; dy = -dy; dz = -dz; }	// point it away from the camera
			const double len = sqrt(dx * dx + dy * dy + dz * dz);
			if (len < 1e-12) return -1;
			// horizontal part in map coordinates: map X = dx, map Y = dz
			const double fwd = (dx * fx + dz * fy) / len;
			const double side = (dx * fy - dz * fx) / len;
			if (fwd <= 1e-6) return -1;
			const double ang = atan2(fabs(side), fwd);
			if (ang > maxang) maxang = ang;
		}
		return maxang * 57.29577951308232;	// radians to degrees
	}
};
