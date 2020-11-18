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
package io.github.opencubicchunks.cubicchunks.cubicgen;

import static io.github.opencubicchunks.cubicchunks.api.util.Coords.blockToCube;
import static io.github.opencubicchunks.cubicchunks.api.util.Coords.cubeToMinBlock;

import com.flowpowered.noise.module.source.Perlin;
import io.github.opencubicchunks.cubicchunks.api.worldgen.CubePrimer;
import io.github.opencubicchunks.cubicchunks.api.world.ICube;
import io.github.opencubicchunks.cubicchunks.api.util.Box;
import io.github.opencubicchunks.cubicchunks.api.util.Coords;
import io.github.opencubicchunks.cubicchunks.api.util.CubePos;
import io.github.opencubicchunks.cubicchunks.api.util.IntRange;
import io.github.opencubicchunks.cubicchunks.api.world.ICubicWorldType;
import io.github.opencubicchunks.cubicchunks.api.worldgen.ICubeGenerator;
import mcp.MethodsReturnNonnullByDefault;
import net.minecraft.init.Blocks;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Vec3i;
import net.minecraft.world.World;
import net.minecraft.world.WorldServer;
import net.minecraft.world.WorldType;

import java.util.HashMap;
import java.util.Map;

import javax.annotation.Nonnull;
import javax.annotation.ParametersAreNonnullByDefault;

@MethodsReturnNonnullByDefault
@ParametersAreNonnullByDefault
public class DebugWorldType extends WorldType implements ICubicWorldType {

    private DebugWorldType() {
        super("DebugCubic");
    }

    @Override
    public boolean canBeCreated() {
        return CustomCubicMod.DEBUG_ENABLED;
    }

    public static DebugWorldType create() {
        return new DebugWorldType();
    }

    @Override public IntRange calculateGenerationHeightRange(WorldServer world) {
        return new IntRange(0, 256);
    }

    @Override public boolean hasCubicGeneratorForWorld(World world) {
        return true;
    }

    @Override public ICubeGenerator createCubeGenerator(World world) {
        return new MultiGridGenerator(world)
                .gridSize(blockToCube(2048))
                .add(0, 0, new LowFrequencyPerlinLightTestGenerator(world))
                .add(5, 0, new Lighting2PlatformsTestGenerator(world));
    }

    private static class MultiGridGenerator extends BasicCubeGenerator {

        private int gridSize = blockToCube(1024);
        // actually used as vec2i
        private Map<Vec3i, ICubeGenerator> generators = new HashMap<>();

        MultiGridGenerator(World world) {
            super(world);
        }

        MultiGridGenerator add(int gridX, int gridZ, ICubeGenerator gen) {
            generators.put(new Vec3i(gridX, 0, gridZ), gen);
            return this;
        }

        MultiGridGenerator gridSize(int gridSizeCubes) {
            this.gridSize = gridSizeCubes;
            return this;
        }

        @Override public CubePrimer generateCube(int cubeX, int cubeY, int cubeZ) {
            ICubeGenerator gen = generators.get(gridPos(cubeX, cubeZ));
            if (gen == null) {
                return new CubePrimer();
            }
            return gen.generateCube(cubeX, cubeY, cubeZ);
        }


        @Override public void populate(ICube cube) {
            ICubeGenerator gen = generators.get(gridPos(cube.getX(), cube.getZ()));
            if (gen != null) {
                gen.populate(cube);
            }
        }

        @Override public Box getFullPopulationRequirements(ICube cube) {
            ICubeGenerator gen = generators.get(gridPos(cube.getX(), cube.getZ()));
            if (gen == null) {
                return NO_REQUIREMENT;
            }
            return gen.getFullPopulationRequirements(cube);
        }

        private Vec3i gridPos(int cubeX, int cubeZ) {
            return new Vec3i(Math.floorDiv(cubeX, gridSize), 0, Math.floorDiv(cubeZ, gridSize));
        }
    }

    private static abstract class NoPopulatorDensityBasedGenerator extends BasicCubeGenerator {

        NoPopulatorDensityBasedGenerator(World world) {
            super(world);
        }

        protected abstract double getDensity(int blockX, int blockY, int blockZ);

        @Override public final CubePrimer generateCube(int cubeX, int cubeY, int cubeZ) {
            CubePrimer primer = new CubePrimer();

            CubePos cubePos = new CubePos(cubeX, cubeY, cubeZ);
            for (BlockPos pos : BlockPos.getAllInBoxMutable(cubePos.getMinBlockPos(), cubePos.getMaxBlockPos())) {
                double currDensity = getDensity(pos.getX(), pos.getY(), pos.getZ());
                double aboveDensity = getDensity(pos.getX(), (pos.getY() + 1), pos.getZ());

                double yGradAbs = Math.abs(aboveDensity - currDensity);

                if (currDensity > 0) {
                    if (aboveDensity <= 0) {
                        primer.setBlockState(Coords.blockToLocal(pos.getX()), Coords.blockToLocal(pos.getY()), Coords.blockToLocal(pos.getZ()),
                                Blocks.GRASS.getDefaultState());
                    } else if (currDensity > aboveDensity && currDensity / yGradAbs <= 4) {
                        primer.setBlockState(Coords.blockToLocal(pos.getX()), Coords.blockToLocal(pos.getY()), Coords.blockToLocal(pos.getZ()),
                                Blocks.DIRT.getDefaultState());
                    } else {
                        primer.setBlockState(Coords.blockToLocal(pos.getX()), Coords.blockToLocal(pos.getY()), Coords.blockToLocal(pos.getZ()),
                                Blocks.STONE.getDefaultState());
                    }
                }
            }
            return primer;
        }

        @Override public final void populate(ICube cube) {
        }
    }

    private static class LowFrequencyPerlinLightTestGenerator extends NoPopulatorDensityBasedGenerator {

        @Nonnull Perlin perlin = new Perlin();

        {
            perlin.setFrequency(0.180);
            perlin.setOctaveCount(1);
            perlin.setSeed((int) world.getSeed());
        }

        public LowFrequencyPerlinLightTestGenerator(World world) {
            super(world);
        }

        @Override protected double getDensity(int blockX, int blockY, int blockZ) {
            if (blockY > cubeToMinBlock(30)) {
                return -1;
            }
            if (blockToCube(blockX) == 100 && blockToCube(blockZ) == 100) {
                return -1; //hole in the world
            }
            double currDensity = perlin.getValue(blockX, blockY * 0.5, blockZ) - 0.5;
            if (blockY >= 256) {
                currDensity -= (blockY - 256) / 100;
            }
            return currDensity;
        }
    }


    private static class Lighting2PlatformsTestGenerator extends NoPopulatorDensityBasedGenerator {

        @Nonnull Perlin perlin = new Perlin();

        {
            perlin.setFrequency(0.180);
            perlin.setOctaveCount(1);
            perlin.setSeed((int) world.getSeed());
        }

        public Lighting2PlatformsTestGenerator(World world) {
            super(world);
        }

        @Override protected double getDensity(int blockX, int blockY, int blockZ) {
            return blockY == 200 || blockY <= 0 ? 1 : -1;
        }
    }

}
