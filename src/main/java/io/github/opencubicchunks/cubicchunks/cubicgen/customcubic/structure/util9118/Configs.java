/*
 *  This file is part of Cubic World Generation, licensed under the MIT License (MIT).
 *
 *  Copyright (c) 2015-2020 contributors
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 */
package io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.structure.util9118;

import net.minecraftforge.common.config.Config;
import net.minecraftforge.common.config.ConfigManager;
import net.minecraftforge.fml.client.event.ConfigChangedEvent;
import net.minecraftforge.fml.common.Mod;
import net.minecraftforge.fml.common.eventhandler.SubscribeEvent;

@Config(modid = Reference.MOD_ID, category = "")
@Mod.EventBusSubscriber(modid = Reference.MOD_ID)
public class Configs 
{
	public static ConfigCaveGen cavegen = new ConfigCaveGen();
	
	public static class ConfigCaveGen
	{
		@Config.Comment({"Controls size of caves. Smaller values = larger caves. Between -1.0 and 1.0", "Default: -0.18"})
		@Config.RequiresWorldRestart
		public double noiseCutoffValue = -0.18;
		
		@Config.Comment({"Controls size of caves at the surface. Smaller values = more caves break through the surface. Between -1.0 and 1.0", "Default: -0.081 (45% of noiseCutoffValue)"})
		@Config.RequiresWorldRestart
		public double surfaceCutoffValue = -0.081;
		
		@Config.Comment({"Controls how much to warp caves. Lower values = straighter caves", "Default: 8.0"})
		@Config.RequiresWorldRestart
		public double warpAmplifier = 8.0;
		
		@Config.Comment({"Reduces number of caves at surface level, becoming more common until caves generate normally X number of blocks below the surface", "Default: 15"})
		@Config.RequiresWorldRestart
		public int easeInDepth = 15;
		
		@Config.Comment({"Squishes caves on the Y axis. Lower values = taller caves and more steep drops", "Default: 2.0"})
		@Config.RequiresWorldRestart
		public double verticalCompressionMultiplier = 2.0;
		
	    @Config.Comment({"Streches (when < 1.0) or compresses (when > 1.0) cave generation along X and Z axis", "Default: 1.0"}) 
	    @Config.RequiresWorldRestart 
	    public double horizonalCompressionMultiplier = 1.0; 
		
		@Config.Comment({"Dimension IDs that will use Vanilla cave generation rather than Worley's Caves", "Default:"})
		@Config.RequiresWorldRestart
		public int[] blackListedDims = {};
		
		@Config.Comment({"Caves will not attempt to generate above this y level. Range whatever the heck you want", "Default: 0"})
		@Config.RequiresWorldRestart
		public int maxCaveHeight = 0;
		
		@Config.Comment({"Block to use when generating large lava lakes below lavaDepth (usually y=10)", "Default: minecraft:lava"})
		@Config.RequiresWorldRestart
		public String lavaBlock = "minecraft:lava";
		
		@Config.Comment({"Air blocks at or below this y level will generate as lavaBlock", "Default: 10"})
		@Config.RequiresWorldRestart
		public int lavaDepth = 10;

		@Config.Comment({"Allow replacing more blocks with caves (useful for mods which completely overwrite world gen)"})
		@Config.RequiresWorldRestart
		public boolean allowReplaceMoreBlocks = true;
	}
	
	@SubscribeEvent
	public static void onConfigReload(ConfigChangedEvent.OnConfigChangedEvent event) 
	{
		if (Reference.MOD_ID.equals(event.getModID()))
			ConfigManager.sync(Reference.MOD_ID, Config.Type.INSTANCE);
	}

}
