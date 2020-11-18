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
package io.github.opencubicchunks.cubicchunks.cubicgen.common.biome;

import com.google.common.collect.Sets;
import io.github.opencubicchunks.cubicchunks.cubicgen.CustomCubicMod;
import mcp.MethodsReturnNonnullByDefault;
import net.minecraft.block.state.IBlockState;
import net.minecraft.init.Blocks;
import net.minecraft.util.ResourceLocation;
import net.minecraft.world.World;

import java.util.Set;

import javax.annotation.ParametersAreNonnullByDefault;

@ParametersAreNonnullByDefault
@MethodsReturnNonnullByDefault
public class TerrainShapeReplacer implements IBiomeBlockReplacer {

    private IBlockState terrainFill;

    public TerrainShapeReplacer(IBlockState terrainFill) {
        this.terrainFill = terrainFill;
    }

    /**
     * Replaces any block with greater than 0 density with stone
     */
    @Override
    public IBlockState getReplacedBlock(IBlockState previousBlock, int x, int y, int z, double dx, double dy, double dz, double density) {
        if (density > 0) {
            return terrainFill;
        }
        return previousBlock;
    }

    public static IBiomeBlockReplacerProvider provider() {
        return new IBiomeBlockReplacerProvider() {
            private final ResourceLocation TERRAIN_FILL_BLOCK = CustomCubicMod.location("terrain_fill_block");

            @Override
            public IBiomeBlockReplacer create(World world, CubicBiome cubicBiome, BiomeBlockReplacerConfig conf) {
                IBlockState terrainFill = conf.getBlockstate(TERRAIN_FILL_BLOCK, Blocks.STONE.getDefaultState());
                return new TerrainShapeReplacer(terrainFill);
            }

            @Override public Set<ConfigOptionInfo> getPossibleConfigOptions() {
                return Sets.newHashSet(
                        new ConfigOptionInfo(TERRAIN_FILL_BLOCK, Blocks.STONE.getDefaultState())
                );
            }
        };
    }
}
