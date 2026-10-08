/*
** p_checkposition.h
**
**
**
**---------------------------------------------------------------------------
**
** Copyright 2025-2026 UZDoom Maintainers and Contributors
**
** SPDX-License-Identifier: GPL-3.0-or-later
**
**---------------------------------------------------------------------------
**
*/

#ifndef P_CHECKPOS_H
#define P_CHECKPOS_H

//============================================================================
//
// Used by P_CheckPosition and P_TryMove in place of the original
// set of global variables.
//
//============================================================================

struct FCheckPosition
{
	// in
	AActor			*thing;
	DVector3		pos;

	// out
	sector_t		*sector;
	double			floorz;
	double			ceilingz;
	double			dropoffz;
	FTextureID		floorpic;
	int				floorterrain;
	sector_t		*floorsector;
	FTextureID		ceilingpic;
	sector_t		*ceilingsector;
	bool			touchmidtex;
	bool			abovemidtex;
	bool			floatok;
	bool			FromPMove;
	line_t			*ceilingline;
	AActor			*stepthing;
	double			modelfloorz;	// highest 3D-model surface under a monster at pos that it doesn't stand on, but would drop onto
	AActor          *blockingMobj;
	// [RH] These are used by PIT_CheckThing and P_XYMovement to apply
	// ripping damage once per tic instead of once per move.
	TMap<AActor*, bool> LastRipped;
	bool			DoRipping;
	bool			portalstep;
	bool			dropoffisportal;
	int				portalgroup;

	int				PushTime;
	TArray<DVector3> collisionPoints;

	double TopThingZ()
	{
		double topZ = pos.Z;
		if (collisionPoints.size())
		{
			for (auto p : collisionPoints)
			{
				topZ = max(topZ, p.Z);
			}
		}
		return topZ;
	}

	FCheckPosition(bool rip=false)
	{
		DoRipping = rip;
		PushTime = 0;
		FromPMove = false;
		modelfloorz = -INFINITY;
	}
};


#endif
