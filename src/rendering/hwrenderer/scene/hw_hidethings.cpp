/*
** hw_hidethings.cpp
**
** hidethings: hides actor or visual thinker classes (and their subclasses)
** or plain particles from the hardware renderer, without touching the game.
** Useful for finding which kind of thing costs frame time, including GPU
** time, by comparing the frame rate with and without it.
**
**---------------------------------------------------------------------------
**
** SPDX-License-Identifier: GPL-3.0-or-later
**
**---------------------------------------------------------------------------
**
*/

#include "hw_hidethings.h"
#include "actor.h"
#include "p_visualthinker.h"
#include "c_dispatch.h"
#include "printf.h"
#include "tarray.h"

namespace HideThings
{
	bool HideActive;
	bool HideParticles;
	static TArray<const PClass *> hidden;

	bool IsHidden(const PClass *cls)
	{
		for (auto h : hidden)
		{
			if (cls->IsDescendantOf(h)) return true;
		}
		return false;
	}
}

CCMD(hidethings)
{
	using namespace HideThings;
	if (argv.argc() < 2)
	{
		Printf("Usage: hidethings <class> | particles | clear  (toggles; subclasses are hidden too)\n");
		if (hidden.Size() == 0 && !HideParticles) Printf("Nothing is hidden.\n");
		for (auto h : hidden) Printf("  hidden: %s\n", h->TypeName.GetChars());
		if (HideParticles) Printf("  hidden: particles\n");
		return;
	}
	if (!stricmp(argv[1], "clear"))
	{
		hidden.Clear();
		HideParticles = false;
		HideActive = false;
		Printf("Nothing is hidden.\n");
		return;
	}
	if (!stricmp(argv[1], "particles"))
	{
		HideParticles = !HideParticles;
		Printf("Particles %s\n", HideParticles ? "hidden" : "shown");
		return;
	}
	const PClass *cls = PClass::FindClass(argv[1]);
	if (cls == nullptr || !(cls->IsDescendantOf(RUNTIME_CLASS(AActor)) || cls->IsDescendantOf(RUNTIME_CLASS(DVisualThinker))))
	{
		Printf("'%s' is not an actor or visual thinker class\n", argv[1]);
		return;
	}
	const unsigned index = hidden.Find(cls);
	if (index < hidden.Size())
	{
		hidden.Delete(index);
		Printf("%s shown\n", cls->TypeName.GetChars());
	}
	else
	{
		hidden.Push(cls);
		Printf("%s hidden (with subclasses)\n", cls->TypeName.GetChars());
	}
	HideActive = hidden.Size() > 0;
}
