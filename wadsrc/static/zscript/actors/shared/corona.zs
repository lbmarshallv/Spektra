class Corona : Actor native
{
	native double AlphaMin;
	native double AlphaMax;
	property AlphaClamp : AlphaMin, AlphaMax;
	
	Default
	{
		RenderRadius 1024.0;
		Corona.AlphaClamp 0.25,1.0;
	}
}
