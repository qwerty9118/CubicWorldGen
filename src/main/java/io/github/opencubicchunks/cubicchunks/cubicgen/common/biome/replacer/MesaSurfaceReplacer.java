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
package io.github.opencubicchunks.cubicchunks.cubicgen.common.biome.replacer;

import static java.lang.Math.abs;

import com.google.common.collect.Sets;
import io.github.opencubicchunks.cubicchunks.cubicgen.ConversionUtils;
import io.github.opencubicchunks.cubicchunks.cubicgen.CustomCubicMod;
import io.github.opencubicchunks.cubicchunks.cubicgen.asm.mixin.common.accessor.IBiomeMesa;
import io.github.opencubicchunks.cubicchunks.cubicgen.cache.HashCacheDoubles;
import io.github.opencubicchunks.cubicchunks.cubicgen.common.biome.BiomeBlockReplacerConfig;
import io.github.opencubicchunks.cubicchunks.cubicgen.common.biome.ConfigOptionInfo;
import io.github.opencubicchunks.cubicchunks.cubicgen.common.biome.IBiomeBlockReplacer;
import io.github.opencubicchunks.cubicchunks.cubicgen.common.biome.IBiomeBlockReplacerProvider;
import io.github.opencubicchunks.cubicchunks.cubicgen.common.biome.CubicBiome;
import io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.builder.IBuilder;
import io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.builder.NoiseSource;
import mcp.MethodsReturnNonnullByDefault;
import net.minecraft.block.Block;
import net.minecraft.block.BlockColored;
import net.minecraft.block.BlockDirt;
import net.minecraft.block.state.IBlockState;
import net.minecraft.init.Blocks;
import net.minecraft.item.EnumDyeColor;
import net.minecraft.util.ResourceLocation;
import net.minecraft.util.math.BlockPos;
import net.minecraft.world.World;
import net.minecraft.world.biome.BiomeMesa;
import net.minecraft.world.gen.NoiseGeneratorPerlin;

import java.util.Arrays;
import java.util.Random;
import java.util.Set;

import javax.annotation.ParametersAreNonnullByDefault;

@ParametersAreNonnullByDefault
@MethodsReturnNonnullByDefault
public class MesaSurfaceReplacer implements IBiomeBlockReplacer {

    private final double mesaDepth;
    private final double heightOffset;
    private final double heightScale;
    private final double waterHeight;

    private final IBuilder depthNoise;

    private final BiomeMesa biomeMesa;

    private final IBlockState[] clayBands;
    private final HashCacheDoubles<BlockPos> clayBandsOffsetNoise;
    private final HashCacheDoubles<BlockPos> pillarNoise;
    private final HashCacheDoubles<BlockPos> pillarRoofNoise;

    protected static final IBlockState STAINED_HARDENED_CLAY = Blocks.STAINED_HARDENED_CLAY.getDefaultState();
    protected static final IBlockState AIR = Blocks.AIR.getDefaultState();
    protected static final IBlockState STONE = Blocks.STONE.getDefaultState();
    protected static final IBlockState COARSE_DIRT = Blocks.DIRT.getDefaultState().withProperty(BlockDirt.VARIANT, BlockDirt.DirtType.COARSE_DIRT);
    protected static final IBlockState GRASS = Blocks.GRASS.getDefaultState();
    protected static final IBlockState HARDENED_CLAY = Blocks.HARDENED_CLAY.getDefaultState();
    protected static final IBlockState ORANGE_STAINED_HARDENED_CLAY = STAINED_HARDENED_CLAY.withProperty(BlockColored.COLOR, EnumDyeColor.ORANGE);


    public MesaSurfaceReplacer(World world, CubicBiome biome, IBuilder builder, double depth, double heightOffset, double heightScale, double waterHeight) {
        this.biomeMesa = (BiomeMesa) biome.getBiome();
        this.mesaDepth = depth;
        this.heightOffset = heightOffset;
        this.heightScale = heightScale;
        this.waterHeight = waterHeight;

        IBiomeMesa mesa = (IBiomeMesa) biomeMesa;

        if (mesa.getClayBands() == null || mesa.getWorldSeed() != world.getSeed()) {
            biomeMesa.generateBands(world.getSeed());
        }
        assert mesa.getClayBands() != null;
        // so that we don't cause issues when we replace clayBands and scrollOffset noise
        mesa.setWorldSeed(world.getSeed());
        this.clayBands = Arrays.copyOf(mesa.getClayBands(), mesa.getClayBands().length);
        this.clayBandsOffsetNoise = HashCacheDoubles.create(
                256, p -> p.getX() * 16 + p.getZ(), p -> mesa.getClayBandsOffsetNoise().getValue(p.getX() / 512.0, p.getZ() / 512.0)
        );

        Random random = new Random(world.getSeed());
        NoiseGeneratorPerlin pillasPerlin = new NoiseGeneratorPerlin(random, 4);
        this.pillarNoise = HashCacheDoubles.create(
                256, p -> p.getX() * 16 + p.getZ(), p -> pillasPerlin.getValue(p.getX(), p.getZ())
        );
        NoiseGeneratorPerlin pillarRoofPerlin = new NoiseGeneratorPerlin(random, 1);
        this.pillarRoofNoise = HashCacheDoubles.create(
                256, p -> p.getX() * 16 + p.getZ(), p -> pillarRoofPerlin.getValue(p.getX(), p.getZ())
        );
        this.depthNoise = builder;
    }

    @Override public IBlockState getReplacedBlock(IBlockState previousBlock, int x, int y, int z, double dx, double dy, double dz, double density) {
        if (density < 0) {
            return previousBlock;
        }
        double depth = depthNoise.get(x, 0, z);
        double origDepthNoise = depth - 3;
        double pillarHeight = getPillarHeightVanilla(x, z, origDepthNoise);
        pillarHeight = convertYFromVanilla(pillarHeight);
        if (y < pillarHeight) {
            // simulate pillar density ORed with te terrain
            density = Math.max(density, pillarHeight - y);
        }

        boolean coarse = Math.cos(origDepthNoise * Math.PI) > 0.0D;

        IBlockState top = STAINED_HARDENED_CLAY;
        IBlockState filler = biomeMesa.fillerBlock;

        if (depth < 0) {
            top = AIR;
            filler = STONE;
        }

        if (y >= waterHeight - 1) {
            IBiomeMesa mesa = (IBiomeMesa) biomeMesa;
            if (mesa.getHasForest() && y >= convertYFromVanilla(86) + depth * 2) {
                top = coarse ? COARSE_DIRT : GRASS;
                filler = getBand(x, y, z);
            } else if (y > waterHeight + 3 + depth) {
                filler = getBand(x, y, z);
                top = coarse ? HARDENED_CLAY : filler;
            } else {
                top = filler = ORANGE_STAINED_HARDENED_CLAY;
            }
        }

        if (density + dy <= 0) { // if air above
            return top;
        }
        double densityAdjusted = density / abs(dy);
        if (densityAdjusted < this.mesaDepth) {
            return filler;
        }
        return previousBlock;
    }

    private double convertYFromVanilla(double y) {
        y = (y - 64.0) / 64.0;
        y *= heightScale;
        y += heightOffset;
        return y;
    }

    private double getPillarHeightVanilla(int x, int z, double depth) {
        double pillarHeight = 0.0;
        IBiomeMesa mesa = (IBiomeMesa) biomeMesa;
        if (mesa.isBrycePillars()) {
            double pillarScale = Math.min(abs(depth),
                    this.pillarNoise.get(new BlockPos(x * 0.25D, 0, z * 0.25D)));

            if (pillarScale > 0.0D) {
                double xzScale = 0.001953125D;
                double pillarRoofVal = abs(this.pillarRoofNoise.get(new BlockPos(x * xzScale, 0, z * xzScale)));
                pillarHeight = pillarScale * pillarScale * 2.5D;
                double cutoffHeight = Math.ceil(pillarRoofVal * 50.0D) + 14.0D;

                if (pillarHeight > cutoffHeight) {
                    pillarHeight = cutoffHeight;
                }

                pillarHeight = pillarHeight + 64.0D;
            }
        }
        return pillarHeight;
    }

    private IBlockState getBand(int blockX, int blockY, int blockZ) {
        int offset = (int) Math.round(this.clayBandsOffsetNoise.get(new BlockPos(blockX, 0, blockX)) * 2.0D);
        return clayBands[(blockY + offset + 64) & 63];
    }


    public static IBiomeBlockReplacerProvider provider() {
        return new IBiomeBlockReplacerProvider() {
            // TODO: add some inheritance to avoid duplicating code. This is mostly copied  from SurfaceDefaultReplacer
            private final ResourceLocation OCEAN_LEVEL = CustomCubicMod.location("water_level");
            private final ResourceLocation DEPTH_NOISE_FACTOR = CustomCubicMod.location("biome_fill_depth_factor");
            private final ResourceLocation DEPTH_NOISE_OFFSET = CustomCubicMod.location("biome_fill_depth_offset");
            private final ResourceLocation DEPTH_NOISE_FREQUENCY = CustomCubicMod.location("biome_fill_noise_freq");
            private final ResourceLocation DEPTH_NOISE_OCTAVES = CustomCubicMod.location("biome_fill_noise_octaves");

            private final ResourceLocation MESA_DEPTH = CustomCubicMod.location("mesa_depth");
            private final ResourceLocation HEIGHT_OFFSET = CustomCubicMod.location("height_offset");
            private final ResourceLocation HEIGHT_SCALE = CustomCubicMod.location("height_scale");

            @Override
            public IBiomeBlockReplacer create(World world, CubicBiome cubicBiome, BiomeBlockReplacerConfig conf) {
                double oceanY = conf.getDouble(OCEAN_LEVEL);

                double factor = conf.getDouble(DEPTH_NOISE_FACTOR);
                double offset = conf.getDouble(DEPTH_NOISE_OFFSET);
                double freq = conf.getDouble(DEPTH_NOISE_FREQUENCY);
                int octaves = (int) conf.getDouble(DEPTH_NOISE_OCTAVES);
                double depth = conf.getDouble(MESA_DEPTH);
                double heightOffset = conf.getDouble(HEIGHT_OFFSET);
                double heightScale = conf.getDouble(HEIGHT_SCALE);

                IBuilder builder = NoiseSource.perlin()
                        .frequency(freq).octaves(octaves).create()
                        .mul(factor).add(offset)
                        .cached2d(256, v -> v.getX() + v.getZ() * 16);
                return new MesaSurfaceReplacer(world, cubicBiome, builder, depth, heightOffset, heightScale, oceanY);
            }

            @Override public Set<ConfigOptionInfo> getPossibleConfigOptions() {
                return Sets.newHashSet(
                        new ConfigOptionInfo(OCEAN_LEVEL, 63.0),
                        // TODO: do it properly, currently this value is just temporary until I figure out the right one
                        // TODO: figure out what the above comment actually means
                        new ConfigOptionInfo(DEPTH_NOISE_FACTOR, ((1 << 3) - 1) / 3.0),
                        new ConfigOptionInfo(DEPTH_NOISE_OFFSET, 3.0),
                        new ConfigOptionInfo(DEPTH_NOISE_FREQUENCY, ConversionUtils.frequencyFromVanilla(0.0625f, 4)),
                        new ConfigOptionInfo(DEPTH_NOISE_OCTAVES, 4.0),
                        new ConfigOptionInfo(MESA_DEPTH, 16.0),
                        new ConfigOptionInfo(HEIGHT_OFFSET, 64.0),
                        new ConfigOptionInfo(HEIGHT_SCALE, 64.0)
                );
            }
        };
    }
}
