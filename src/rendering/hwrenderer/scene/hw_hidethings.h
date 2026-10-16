/*
** hw_hidethings.h
**
** hidethings [class|particles|clear]: hides actor or visual thinker classes
** (and their subclasses) or plain particles from the hardware renderer,
** for A/B performance tests. Render-only; the game is unaffected.
**
**---------------------------------------------------------------------------
**
** SPDX-License-Identifier: GPL-3.0-or-later
**
**---------------------------------------------------------------------------
**
*/

#pragma once

class PClass;

namespace HideThings
{
	extern bool HideActive;		// at least one class is hidden
	extern bool HideParticles;	// hidethings particles

	bool IsHidden(const PClass *cls);
}
