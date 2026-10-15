//===========================================================================
//
// FoliageSpawner
//
// Spawns plants picked at random from a weighted list, scattered over a
// disc around itself, then removes itself. The plants go through
// LevelLocals.SpawnFoliage, so qualifying FOLIAGE classes end up in static
// foliage clusters.
//
// Multiplayer and savegames: all randomness comes from Level.FoliageRandom
// (a hash of the spawner's position and seed), never from the game's random
// number generators, so every machine and every load produces exactly the
// same plants. Spawners placed in the map run during map load and their
// plants are part of the map; spawners created later add plants that are
// saved with the level.
//
// Usage:
//
//   class MeadowSpawner : FoliageSpawner
//   {
//       Default
//       {
//           FoliageSpawner.Choices "TallGrass:4, ShortGrass:4, Fern:1, Flowers:0.5";
//           FoliageSpawner.Count 40;
//           FoliageSpawner.Spread 256;
//       }
//   }
//
// Map thing arguments override the defaults: arg0 = count, arg1 = spread
// radius, arg2 = seed (vary it to get a different layout at the same spot).
// A weight of 0 or less means "never". Choices without a weight count as 1.
//
//===========================================================================

class FoliageSpawner : Actor
{
	String FoliageChoices;
	int FoliageCount;
	double FoliageSpread;
	int FoliageSeed;

	property Choices: FoliageChoices;
	property Count: FoliageCount;
	property Spread: FoliageSpread;
	property Seed: FoliageSeed;

	Default
	{
		FoliageSpawner.Count 1;
		FoliageSpawner.Spread 0;
		FoliageSpawner.Seed 0;
		+NOINTERACTION
		+NOBLOCKMAP
		+NOSECTOR
		+DONTSPLASH
		+NOTONAUTOMAP
	}

	States
	{
	Spawn:
		TNT1 A -1;
		Stop;
	}

	override void BeginPlay()
	{
		Super.BeginPlay();
		// For map things the arguments are already set here (unless the class defines its own args).
		if (args[0] > 0) FoliageCount = args[0];
		if (args[1] > 0) FoliageSpread = args[1];
		if (args[2] != 0) FoliageSeed = args[2];
		SpawnPlants();
		Destroy();
	}

	// Parses "Class[:weight], Class[:weight], ..." into parallel arrays.
	protected void ParseChoices(out Array<Class<Actor> > types, out Array<double> weights)
	{
		types.Clear();
		weights.Clear();
		Array<String> entries;
		FoliageChoices.Split(entries, ",", TOK_SKIPEMPTY);
		for (int i = 0; i < entries.Size(); i++)
		{
			String entry = entries[i];
			entry.StripLeftRight();
			if (entry.Length() == 0) continue;

			double weight = 1;
			int colon = entry.IndexOf(":");
			if (colon >= 0)
			{
				String w = entry.Mid(colon + 1);
				w.StripLeftRight();
				weight = w.ToDouble();
				entry = entry.Left(colon);
				entry.StripLeftRight();
			}
			Class<Actor> type = entry;
			if (type == null)
			{
				Console.Printf("\cG%s: unknown class '%s' in Choices", GetClassName(), entry);
				continue;
			}
			if (weight <= 0) continue;
			types.Push(type);
			weights.Push(weight);
		}
	}

	// Override this to place plants differently; keep using Level.FoliageRandom for anything random.
	virtual void SpawnPlants()
	{
		Array<Class<Actor> > types;
		Array<double> weights;
		ParseChoices(types, weights);
		if (types.Size() == 0 || FoliageCount <= 0) return;

		double total = 0;
		for (int i = 0; i < weights.Size(); i++) total += weights[i];

		Vector3 seedPos = (Pos.XY, FoliageSeed);
		for (int n = 0; n < FoliageCount; n++)
		{
			int salt = n * 4 + 1000;

			// Weighted pick.
			double pick = Level.FoliageRandom(seedPos, salt) * total;
			int choice = types.Size() - 1;
			for (int i = 0; i < weights.Size(); i++)
			{
				pick -= weights[i];
				if (pick < 0) { choice = i; break; }
			}

			// Uniform point on the disc.
			Vector2 spot = Pos.XY;
			if (FoliageSpread > 0)
			{
				double r = FoliageSpread * sqrt(Level.FoliageRandom(seedPos, salt + 1));
				double a = 360. * Level.FoliageRandom(seedPos, salt + 2);
				spot += (cos(a), sin(a)) * r;
			}
			Sector sec = Level.PointInSector(spot);
			if (!Level.IsPointInLevel((spot, sec.floorplane.ZAtPoint(spot) + 1))) continue;

			double angle = Angle;
			if (FoliageSpread > 0) angle += 360. * Level.FoliageRandom(seedPos, salt + 3);	// FOLIAGERANDOMYAW classes add their own on top
			Level.SpawnFoliage(types[choice], (spot, 0), angle);
		}
	}
}
