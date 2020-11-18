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

import java.lang.reflect.Field;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

import javax.annotation.Nullable;
import javax.annotation.ParametersAreNonnullByDefault;

import io.github.opencubicchunks.cubicchunks.api.util.Box;
import io.github.opencubicchunks.cubicchunks.api.util.Coords;
import io.github.opencubicchunks.cubicchunks.api.util.CubePos;
import io.github.opencubicchunks.cubicchunks.api.world.ICube;
import io.github.opencubicchunks.cubicchunks.api.world.ICubeProvider;
import io.github.opencubicchunks.cubicchunks.api.worldgen.ICubeGenerator;
import mcp.MethodsReturnNonnullByDefault;
import net.minecraft.entity.EnumCreatureType;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.ChunkPos;
import net.minecraft.world.World;
import net.minecraft.world.biome.Biome;
import net.minecraft.world.biome.Biome.SpawnListEntry;
import net.minecraft.world.chunk.Chunk;
import net.minecraft.world.chunk.IChunkProvider;
import net.minecraftforge.fml.common.IWorldGenerator;
import net.minecraftforge.fml.common.registry.GameRegistry;

import java.util.HashSet;
import java.util.List;
import java.util.Random;

import javax.annotation.Nullable;
import javax.annotation.ParametersAreNonnullByDefault;

/**
 * A partial implementation of {@link ICubeGenerator} that handles biome assignment.
 * <p>
 * Structure recreation and lookup are not supported by default.
 */
@ParametersAreNonnullByDefault
@MethodsReturnNonnullByDefault
public abstract class BasicCubeGenerator implements ICubeGenerator {

    protected World world;
    private Biome[] columnBiomes;

    public BasicCubeGenerator(World world) {
        this.world = world;
    }
    
    
    HashSet<Long> queuedChunks = new HashSet<Long>();

    /**
     * Called By Generators who want to add support for full chunk populating
     * @param rand
     * @param x
     * @param z
     */
    public void populateChunk(Random rand, int x, int z) {
        Long posLong = ChunkPos.asLong(x,z);
        
        
        Field worldGeneratorsx = null;
		try {
			worldGeneratorsx = GameRegistry.class.getDeclaredField("worldGenerators");
		} catch (NoSuchFieldException | SecurityException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
        worldGeneratorsx.setAccessible(true);
        Set<IWorldGenerator> worldGenerators = null;
		try {
			worldGenerators = (Set<IWorldGenerator>) worldGeneratorsx.get(worldGeneratorsx);
		} catch (IllegalArgumentException | IllegalAccessException e) {
			e.printStackTrace();
		}
        
        
//		if (!queuedChunks.contains(posLong) && !GameRegistry.worldGenerators.isEmpty())
		if (!queuedChunks.contains(posLong) && !worldGenerators.isEmpty())
        {
            IChunkProvider provider = world.getChunkProvider();
            if (provider instanceof ICubeProvider) {
                queuedChunks.add(posLong);
                //Generate the Entire Column
                for (int y = 0; y < 16; y++)
                {
                    ((ICubeProvider) provider).getCube(x, y, z);
                }
            }
//            for (IWorldGenerator worldGenerator : GameRegistry.worldGenerators)
              for (IWorldGenerator worldGenerator : worldGenerators)
            {
                worldGenerator.generate(rand, x, z, world, null, world.getChunkProvider());
            }
            queuedChunks.remove(posLong);
        }
    }
    

    @Override
    public void generateColumn(Chunk column) {
        this.columnBiomes = this.world.getBiomeProvider()
                .getBiomes(this.columnBiomes,
                        Coords.cubeToMinBlock(column.x),
                        Coords.cubeToMinBlock(column.z),
                        ICube.SIZE, ICube.SIZE);

        // Copy ids to column internal biome array
        byte[] columnBiomeArray = column.getBiomeArray();
        for (int i = 0; i < columnBiomeArray.length; ++i) {
            columnBiomeArray[i] = (byte) Biome.getIdForBiome(this.columnBiomes[i]);
        }
    }

    @Override
    public void recreateStructures(ICube cube) {
    }

    @Override
    public void recreateStructures(Chunk column) {
    }

    @Override
    public List<SpawnListEntry> getPossibleCreatures(EnumCreatureType type, BlockPos pos) {
        return world.getBiome(pos).getSpawnableList(type);
    }

    @Nullable @Override
    public BlockPos getClosestStructure(String name, BlockPos pos, boolean findUnexplored) {
        return null;
    }

    @Override
    public Box getFullPopulationRequirements(ICube cube) {
        return RECOMMENDED_FULL_POPULATOR_REQUIREMENT;
    }

    @Override
    public Box getPopulationPregenerationRequirements(ICube cube) {
        return RECOMMENDED_GENERATE_POPULATOR_REQUIREMENT;
    }

}
