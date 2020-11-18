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
package io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.populator;

import io.github.opencubicchunks.cubicchunks.api.worldgen.populator.ICubicPopulator;
import io.github.opencubicchunks.cubicchunks.api.util.CubePos;
import io.github.opencubicchunks.cubicchunks.cubicgen.CWGEventFactory;
import io.github.opencubicchunks.cubicchunks.cubicgen.asm.mixin.common.accessor.IBiome;
import io.github.opencubicchunks.cubicchunks.cubicgen.asm.mixin.common.accessor.IBiomePlains;
import mcp.MethodsReturnNonnullByDefault;
import net.minecraft.block.BlockDoublePlant;
import net.minecraft.world.World;
import net.minecraft.world.biome.Biome;
import net.minecraft.world.biome.BiomeDecorator;
import net.minecraft.world.biome.BiomePlains;
import net.minecraftforge.event.terraingen.DecorateBiomeEvent;

import java.util.Random;

import javax.annotation.ParametersAreNonnullByDefault;

@ParametersAreNonnullByDefault
@MethodsReturnNonnullByDefault
public class PlainsDecorator implements ICubicPopulator {

    @Override public void generate(World world, Random random, CubePos pos, Biome biome) {
        double randomValue = IBiome.getGrassColorNoise().getValue((double) (pos.getX() + 8) / 200.0D, (double) (pos.getZ() + 8) / 200.0D);

        BiomeDecorator dec = biome.decorator;
        if (randomValue < -0.8D) {
            dec.flowersPerChunk = 15;
            dec.grassPerChunk = 5;
        } else {
            dec.flowersPerChunk = 4;
            dec.grassPerChunk = 10;
            IBiome.getDoublePlantGenerator().setPlantType(BlockDoublePlant.EnumPlantType.GRASS);

            if (CWGEventFactory.decorate(world, random, pos, DecorateBiomeEvent.Decorate.EventType.GRASS)) {
                for (int i = 0; i < 7; ++i) {
                    // see: DefaultDecorator, flower generator
                    if (random.nextInt(7) != 0) {
                        continue;
                    }
                    IBiome.getDoublePlantGenerator().generate(world, random, pos.randomPopulationPos(random));
                }
            }
        }

        if (((IBiomePlains) biome).isSunflowers() && CWGEventFactory.decorate(world, random, pos, DecorateBiomeEvent.Decorate.EventType.FLOWERS)) {
            IBiome.getDoublePlantGenerator().setPlantType(BlockDoublePlant.EnumPlantType.SUNFLOWER);

            for (int i = 0; i < 10; ++i) {
                if (random.nextInt(7) != 0) {
                    continue;
                }
                IBiome.getDoublePlantGenerator().generate(world, random, pos.randomPopulationPos(random));
            }
        }
    }
}
