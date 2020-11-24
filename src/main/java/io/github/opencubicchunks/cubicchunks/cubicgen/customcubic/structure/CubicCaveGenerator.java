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
package io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.structure;

import static io.github.opencubicchunks.cubicchunks.api.util.Coords.cubeToMinBlock;
import static io.github.opencubicchunks.cubicchunks.api.util.Coords.localToBlock;
import static io.github.opencubicchunks.cubicchunks.cubicgen.StructureGenUtil.normalizedDistance;
import static java.lang.Math.max;
import static net.minecraft.util.math.MathHelper.cos;
import static net.minecraft.util.math.MathHelper.floor;
import static net.minecraft.util.math.MathHelper.sin;

import java.util.Random;
import java.util.function.Predicate;

import javax.annotation.Nonnull;
import javax.annotation.ParametersAreNonnullByDefault;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import com.google.common.base.MoreObjects;

import io.github.opencubicchunks.cubicchunks.api.util.CubePos;
import io.github.opencubicchunks.cubicchunks.api.world.ICube;
import io.github.opencubicchunks.cubicchunks.api.worldgen.CubePrimer;
import io.github.opencubicchunks.cubicchunks.api.worldgen.structure.ICubicStructureGenerator;
import io.github.opencubicchunks.cubicchunks.cubicgen.StructureGenUtil;
import io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.structure.util9118.BlockUtil;
import io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.structure.util9118.Configs;
import io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.structure.util9118.FastNoise;
import io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.structure.util9118.Reference;
import io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.structure.util9118.WorleyUtil;
import mcp.MethodsReturnNonnullByDefault;
import net.minecraft.block.Block;
import net.minecraft.block.BlockLiquid;
import net.minecraft.block.material.Material;
import net.minecraft.block.state.IBlockState;
import net.minecraft.init.Biomes;
import net.minecraft.init.Blocks;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.MathHelper;
import net.minecraft.world.World;
import net.minecraft.world.biome.Biome;
import net.minecraft.world.gen.MapGenBase;
import net.minecraft.world.gen.MapGenCaves;
import net.minecraft.world.gen.structure.StructureBoundingBox;
import net.minecraftforge.fluids.IFluidBlock;
import net.minecraftforge.fml.common.Loader;

/*
 * Modified Minecraft cave generation code. Based on Robinton's cave generation implementation.
 */
//TODO: Fix code duplication beterrn cave and cave generators
@ParametersAreNonnullByDefault
@MethodsReturnNonnullByDefault
public class CubicCaveGenerator implements ICubicStructureGenerator {

    //=============================================
    //Possibly configurable values
    //=============================================

    /**
     * 1 in CAVE_RARITY attempts will result in generating any caves at all
     * <p>
     * Vanilla value: 7 Multiply by 16 and divide by 8: there are 16 cubes in a vanilla chunk, but only one cube per 8
     * has caves generated
     */
    private static final int CAVE_RARITY = 16 * 7 / (2 * 2 * 2);

    /**
     * Maximum amount of starting nodes
     */
    private static final int MAX_INIT_NODES = 14;

    /**
     * 1 in LARGE_NODE_RARITY initial attempts will result in large node
     */
    private static final int LARGE_NODE_RARITY = 4;

    /**
     * The maximum amount of additional branches after generating large node. Random value between 0 and
     * LARGE_NODE_MAX_BRANCHES is chosen.
     */
    private static final int LARGE_NODE_MAX_BRANCHES = 4;

    /**
     * 1 in BIG_CAVE_RARITY branches will start bigger than usual
     */
    private static final int BIG_CAVE_RARITY = 10;

    /**
     * Value added to the size of the cave (radius)
     */
    private static final double CAVE_SIZE_ADD = 1.5D;

    /**
     * In 1 of STEEP_STEP_RARITY steps, cave will be flattened using STEEPER_FLATTEN_FACTOR instead of FLATTEN_FACTOR
     */
    private static final int STEEP_STEP_RARITY = 6;

    /**
     * After each step the Y direction component will be multiplied by this value, unless steeper cave is allowed
     */
    private static final float FLATTEN_FACTOR = 0.7f;

    /**
     * If steeper cave is allowed - this value will be used instead of FLATTEN_FACTOR
     */
    private static final float STEEPER_FLATTEN_FACTOR = 0.92f;

    /**
     * Each step cave direction angles will be changed by this fraction of values that specify how direction changes
     */
    private static final float DIRECTION_CHANGE_FACTOR = 0.1f;

    /**
     * This fraction of the previous value that controls horizontal direction changes will be used in next step
     */
    private static final float PREV_HORIZ_DIRECTION_CHANGE_WEIGHT = 0.75f;

    /**
     * This fraction of the previous value that controls vertical direction changes will be used in next step
     */
    private static final float PREV_VERT_DIRECTION_CHANGE_WEIGHT = 0.9f;

    /**
     * Maximum value by which horizontal cave direction randomly changes each step, lower values are much more likely.
     */
    private static final float MAX_ADD_DIRECTION_CHANGE_HORIZ = 4.0f;

    /**
     * Maximum value by which vertical cave direction randomly changes each step, lower values are much more likely.
     */
    private static final float MAX_ADD_DIRECTION_CHANGE_VERT = 2.0f;

    /**
     * 1 in this amount of steps will actually carve any blocks,
     */
    private static final int CARVE_STEP_RARITY = 4;

    /**
     * Relative "height" if depth floor
     * <p>
     * -1 results in round cave without flat floor 1 will completely fill the cave 0 will result in lower half of the
     * cave to be filled with stone
     */
    private static final double CAVE_FLOOR_DEPTH = -0.7;

    private static final int RANGE = 8;
    
    /**
     * 9118 - Maximum height that a cave will start generating at.
     */
    private static final long MAX_HEIGHT = -2000;

    /**
     * Controls which blocks can be replaced by cave
     */
    private static final Predicate<IBlockState> isBlockReplaceable = (state ->
            state.getBlock() == Blocks.STONE || state.getBlock() == Blocks.DIRT || state.getBlock() == Blocks.GRASS);

    @Override public void generate(World world, CubePrimer cube, CubePos cubePos) {
    	if(!worlInit) {
        	WorleyCaveGenerator();
        	worlInit = true;
    	}
    	
    	generate2(world, cube, cubePos);
    	this.generate(world, cube, cubePos, this::generate, RANGE, RANGE, 1, 1);
    }
    
    boolean worlInit = false;

    protected void generate(World world, Random rand, CubePrimer cube,
            int cubeXOrigin, int cubeYOrigin, int cubeZOrigin, CubePos generatedCubePos) {
        if (rand.nextInt(CAVE_RARITY) != 0 || generatedCubePos.getY() > MAX_HEIGHT/16) {
            return;
        }
        //very low probability of generating high number //TODO: 9118 - do the 1.6.4 caves thing (or possibly above) (i think below is for the size of the caves, above is for chance of spawning a cave.
        int nodes = rand.nextInt(rand.nextInt(rand.nextInt(MAX_INIT_NODES + 1) + 1) + 1);

        for (int node = 0; node < nodes; ++node) {
            double branchStartX = localToBlock(cubeXOrigin, rand.nextInt(ICube.SIZE));
            double branchStartY = localToBlock(cubeYOrigin, rand.nextInt(ICube.SIZE));
            double branchStartZ = localToBlock(cubeZOrigin, rand.nextInt(ICube.SIZE));
            int subBranches = 1;

            if (rand.nextInt(LARGE_NODE_RARITY) == 0) {
                this.generateLargeNode(cube, rand, rand.nextLong(), generatedCubePos,
                        branchStartX, branchStartY, branchStartZ);
                subBranches += rand.nextInt(LARGE_NODE_MAX_BRANCHES);
            }

            for (int branch = 0; branch < subBranches; ++branch) {
                float horizDirAngle = rand.nextFloat() * (float) Math.PI * 2.0F;
                float vertDirAngle = (rand.nextFloat() - 0.5F) * 2.0F / 8.0F;
                float baseHorizSize = rand.nextFloat() * 2.0F + rand.nextFloat();

                if (rand.nextInt(BIG_CAVE_RARITY) == 0) {
                    baseHorizSize *= rand.nextFloat() * rand.nextFloat() * 3.0F + 1.0F;
                }

                int startWalkedDistance = 0;
                int maxWalkedDistance = 0;
                double vertCaveSizeMod = 1.0;

                this.generateNode(cube, rand.nextLong(), generatedCubePos,
                        branchStartX, branchStartY, branchStartZ,
                        baseHorizSize, horizDirAngle, vertDirAngle,
                        startWalkedDistance, maxWalkedDistance, vertCaveSizeMod);
            }
        }
    }

    /**
     * Generates a flattened cave "room", usually more caves split off it
     */
    private void generateLargeNode(CubePrimer cube, Random rand, long seed, CubePos generatedCubePos,
            double x, double y, double z) {
        float baseHorizSize = 1.0F + rand.nextFloat() * 6.0F;
        float horizDirAngle = 0;
        float vertDirAngle = 0;

        int startWalkedDistance = -1;
        int maxWalkedDistance = -1;
        double vertCaveSizeMod = 0.5;
        this.generateNode(cube, seed, generatedCubePos, x, y, z,
                baseHorizSize, horizDirAngle, vertDirAngle,
                startWalkedDistance, maxWalkedDistance, vertCaveSizeMod);
    }

    /**
     * Recursively generates a node in the current cave system tree.
     *
     * @param cube block buffer to modify
     * @param seed random seed to use
     * @param generatedCubePos position of the cube to modify
     * @param caveX starting x coordinate of the cave
     * @param caveY starting Y coordinate of the cave
     * @param caveZ starting Z coordinate of the cave
     * @param baseCaveSize initial value for cave size, size decreases as cave goes further
     * @param horizDirAngle horizontal direction angle
     * @param vertCaveSizeMod vertical direction angle
     * @param startWalkedDistance the amount of steps the cave already went forwards, used in recursive step. -1 means
     * that there will be only one step
     * @param maxWalkedDistance maximum distance the cave can go forwards, <= 0 to use default
     * @param vertDirAngle changes vertical size of the cave, values < 1 result in flattened caves, > 1 result in
     * vertically stretched caves
     */
    private void generateNode(CubePrimer cube, long seed,
            CubePos generatedCubePos,
            double caveX, double caveY, double caveZ,
            float baseCaveSize, float horizDirAngle, float vertDirAngle,
            int startWalkedDistance, int maxWalkedDistance, double vertCaveSizeMod) {
        Random rand = new Random(seed);

        //store by how much the horizontal and vertical direction angles will change each step
        float horizDirChange = 0.0F;
        float vertDirChange = 0.0F;

        if (maxWalkedDistance <= 0) {
            int maxBlockRadius = cubeToMinBlock(RANGE - 1);
            maxWalkedDistance = maxBlockRadius - rand.nextInt(maxBlockRadius / 4);
        }

        //if true - this branch won't generate new sub-branches
        boolean finalStep = false;

        int walkedDistance;
        if (startWalkedDistance == -1) {
            //generate a cave "room"
            //start at half distance towards the end = max cave size
            walkedDistance = maxWalkedDistance / 2;
            finalStep = true;
        } else {
            walkedDistance = startWalkedDistance;
        }

        int splitPoint = rand.nextInt(maxWalkedDistance / 2) + maxWalkedDistance / 4;

        for (; walkedDistance < maxWalkedDistance; ++walkedDistance) {
            float fractionWalked = walkedDistance / (float) maxWalkedDistance;
            //horizontal and vertical size of the cave
            //size starts small and increases, then decreases as cave goes further
            double caveSizeHoriz = CAVE_SIZE_ADD + sin(fractionWalked * (float) Math.PI) * baseCaveSize;
            double caveSizeVert = caveSizeHoriz * vertCaveSizeMod;

            //Walk forward a single step:

            //from sin(alpha)=y/r and cos(alpha)=x/r ==> x = r*cos(alpha) and y = r*sin(alpha)
            //always moves by one block in some direction

            //here x is xzDirectionFactor, y is yDirectionFactor
            float xzDirectionFactor = cos(vertDirAngle);
            float yDirectionFactor = sin(vertDirAngle);

            //here y is directionZ and x is directionX
            caveX += cos(horizDirAngle) * xzDirectionFactor;
            caveY += yDirectionFactor;
            caveZ += sin(horizDirAngle) * xzDirectionFactor;

            if (rand.nextInt(STEEP_STEP_RARITY) == 0) {
                vertDirAngle *= STEEPER_FLATTEN_FACTOR;
            } else {
                vertDirAngle *= FLATTEN_FACTOR;
            }

            //change the direction
            vertDirAngle += vertDirChange * DIRECTION_CHANGE_FACTOR;
            horizDirAngle += horizDirChange * DIRECTION_CHANGE_FACTOR;
            //update direction change angles
            vertDirChange *= PREV_VERT_DIRECTION_CHANGE_WEIGHT;
            horizDirChange *= PREV_HORIZ_DIRECTION_CHANGE_WEIGHT;
            vertDirChange += (rand.nextFloat() - rand.nextFloat()) * rand.nextFloat() * MAX_ADD_DIRECTION_CHANGE_VERT;
            horizDirChange += (rand.nextFloat() - rand.nextFloat()) * rand.nextFloat() * MAX_ADD_DIRECTION_CHANGE_HORIZ;

            //if we reached split point - try to split
            //can split only if it's not final branch and the cave is still big enough (>1 block radius)
            if (!finalStep && walkedDistance == splitPoint && baseCaveSize > 1.0F) {
                this.generateNode(cube, rand.nextLong(),
                        generatedCubePos, caveX, caveY, caveZ,
                        rand.nextFloat() * 0.5F + 0.5F,//base cave size
                        horizDirAngle - ((float) Math.PI / 2F),//horiz. angle - subtract 90 degrees
                        vertDirAngle / 3.0F, walkedDistance, maxWalkedDistance,
                        1.0D);
                this.generateNode(cube, rand.nextLong(), generatedCubePos, caveX, caveY, caveZ,
                        rand.nextFloat() * 0.5F + 0.5F,//base cave size
                        horizDirAngle + ((float) Math.PI / 2F),//horiz. angle - add 90 degrees
                        vertDirAngle / 3.0F, walkedDistance, maxWalkedDistance,
                        1.0D);
                return;
            }

            //carve blocks only on some percentage of steps, unless this is the final branch
            if (rand.nextInt(CARVE_STEP_RARITY) == 0 && !finalStep) {
                continue;
            }

            double xDist = caveX - generatedCubePos.getXCenter();
            double yDist = caveY - generatedCubePos.getYCenter();
            double zDist = caveZ - generatedCubePos.getZCenter();
            double maxStepsDist = maxWalkedDistance - walkedDistance;
            //CHANGE: multiply max(1, vertCaveSizeMod)
            double maxDistToCube = baseCaveSize * max(1, vertCaveSizeMod) + CAVE_SIZE_ADD + ICube.SIZE;

            //can this cube be reached at all?
            //if even after going max distance allowed by remaining steps, it's still too far - stop
            //TODO: does it make any performance difference?
            if (xDist * xDist + yDist * yDist + zDist * zDist - maxStepsDist * maxStepsDist > maxDistToCube * maxDistToCube) {
                return;
            }

            tryCarveBlocks(cube, generatedCubePos,
                    caveX, caveY, caveZ,
                    caveSizeHoriz, caveSizeVert);
            if (finalStep) {
                return;
            }
        }
    }

    //returns true if cave generation should be continued
    private void tryCarveBlocks(@Nonnull CubePrimer cube, @Nonnull CubePos generatedCubePos,
            double caveX, double caveY, double caveZ,
            double caveSizeHoriz, double caveSizeVert) {
        double genCubeCenterX = generatedCubePos.getXCenter();
        double genCubeCenterY = generatedCubePos.getYCenter();
        double genCubeCenterZ = generatedCubePos.getZCenter();

        //Can current step position affect currently modified cube?
        //TODO: is multiply by 2 needed?
        if (caveX < genCubeCenterX - ICube.SIZE - caveSizeHoriz * 2.0D ||
                caveY < genCubeCenterY - ICube.SIZE - caveSizeVert * 2.0D ||
                caveZ < genCubeCenterZ - ICube.SIZE - caveSizeHoriz * 2.0D ||
                caveX > genCubeCenterX + ICube.SIZE + caveSizeHoriz * 2.0D ||
                caveY > genCubeCenterY + ICube.SIZE + caveSizeVert * 2.0D ||
                caveZ > genCubeCenterZ + ICube.SIZE + caveSizeHoriz * 2.0D) {
            return;
        }
        int minLocalX = floor(caveX - caveSizeHoriz) - generatedCubePos.getMinBlockX() - 1;
        int maxLocalX = floor(caveX + caveSizeHoriz) - generatedCubePos.getMinBlockX() + 1;
        int minLocalY = floor(caveY - caveSizeVert) - generatedCubePos.getMinBlockY() - 1;
        int maxLocalY = floor(caveY + caveSizeVert) - generatedCubePos.getMinBlockY() + 1;
        int minLocalZ = floor(caveZ - caveSizeHoriz) - generatedCubePos.getMinBlockZ() - 1;
        int maxLocalZ = floor(caveZ + caveSizeHoriz) - generatedCubePos.getMinBlockZ() + 1;

        //skip is if everything is outside of that cube
        if (maxLocalX <= 0 || minLocalX >= ICube.SIZE ||
                maxLocalY <= 0 || minLocalY >= ICube.SIZE ||
                maxLocalZ <= 0 || minLocalZ >= ICube.SIZE) {
            return;
        }
        StructureBoundingBox boundingBox = new StructureBoundingBox(minLocalX, minLocalY, minLocalZ, maxLocalX, maxLocalY, maxLocalZ);

        StructureGenUtil.clampBoundingBoxToLocalCube(boundingBox);

        boolean hitLiquid = StructureGenUtil.scanWallsForBlock(cube, boundingBox,
                (b) -> b.getBlock() == Blocks.LAVA || b.getBlock() == Blocks.FLOWING_LAVA);

        if (!hitLiquid) {
            carveBlocks(cube, generatedCubePos, caveX, caveY, caveZ, caveSizeHoriz, caveSizeVert, boundingBox);
        }
    }

    private void carveBlocks(CubePrimer cube,
            CubePos generatedCubePos,
            double caveX, double caveY, double caveZ,
            double caveSizeHoriz, double caveSizeVert,
            StructureBoundingBox boundingBox) {

        int generatedCubeX = generatedCubePos.getX();
        int generatedCubeY = generatedCubePos.getY();
        int generatedCubeZ = generatedCubePos.getZ();

        int minX = boundingBox.minX;
        int maxX = boundingBox.maxX;
        int minY = boundingBox.minY;
        int maxY = boundingBox.maxY;
        int minZ = boundingBox.minZ;
        int maxZ = boundingBox.maxZ;

        for (int localX = minX; localX < maxX; ++localX) {
            double distX = normalizedDistance(generatedCubeX, localX, caveX, caveSizeHoriz);

            for (int localZ = minZ; localZ < maxZ; ++localZ) {
                double distZ = normalizedDistance(generatedCubeZ, localZ, caveZ, caveSizeHoriz);

                if (distX * distX + distZ * distZ >= 1.0D) {
                    continue;
                }
                for (int localY = minY; localY < maxY; ++localY) {
                    double distY = normalizedDistance(generatedCubeY, localY, caveY, caveSizeVert);

                    IBlockState state = cube.getBlockState(localX, localY, localZ);

                    if (!isBlockReplaceable.test(state)) {
                        continue;
                    }

                    if (shouldCarveBlock(distX, distY, distZ)) {
                        // No lava generation, infinite depth. Lava will be generated differently (or not generated)
                        cube.setBlockState(localX, localY, localZ, Blocks.AIR.getDefaultState());
                    } else if (state.getBlock() == Blocks.DIRT) {
                        //vanilla dirt-grass replacement works by scanning top-down and moving the block
                        //cubic chunks needs to be a bit more hacky about it
                        //instead of keeping track of the encountered grass block
                        //cubic chunks replaces any dirt block (it's before population, no ore-like dirt formations yet)
                        //with grass, if the block above would be deleted by this cave generator step
                        double distYAbove = normalizedDistance(generatedCubeY, localY + 1, caveY, caveSizeVert);
                        if (shouldCarveBlock(distX, distYAbove, distZ)) {
                            cube.setBlockState(localX, localY, localZ, Blocks.GRASS.getDefaultState());
                        }
                    }
                }
            }
        }
    }

    private static boolean shouldCarveBlock(double distX, double distY, double distZ) {
        //distY > CAVE_FLOOR_DEPTH --> flattened floor
        return distY > CAVE_FLOOR_DEPTH && distX * distX + distY * distY + distZ * distZ < 1.0D;
    }
    
    
    
    
    
    //-----------------------------------[ code from WorleyCaveGenerator.java + some other worley places ]-----------------------------------
    Logger logger = LogManager.getLogger(Reference.MOD_ID);
    
    int numLogChunks = 500;
	long[] genTime = new long[numLogChunks];
	int currentTimeIndex = 0;
	double sum = 0;

	private WorleyUtil worleyF1divF3 = new WorleyUtil();
	private FastNoise displacementNoisePerlin = new FastNoise();
	private MapGenBase replacementCaves;
	private MapGenBase moddedCaveGen;
	
	
	
	private static IBlockState lava;
	private static final IBlockState SAND = Blocks.SAND.getDefaultState();
	private static final IBlockState RED_SAND = Blocks.SAND.getStateFromMeta(1);
    protected static final IBlockState BLK_LAVA = Blocks.LAVA.getDefaultState();
    protected static final IBlockState BLK_AIR = Blocks.AIR.getDefaultState();
    protected static final IBlockState BLK_SANDSTONE = Blocks.SANDSTONE.getDefaultState();
    protected static final IBlockState BLK_RED_SANDSTONE = Blocks.RED_SANDSTONE.getDefaultState();
    protected static final IBlockState STONE = Blocks.STONE.getDefaultState();
	private static int maxCaveHeight;
	private static float noiseCutoff;
	private static float warpAmplifier;
	private static float easeInDepth;
	private static float yCompression;
	private static float xzCompression;
	private static float surfaceCutoff;
	private static int lavaDepth;
	private static boolean additionalWaterChecks;
	private static int HAS_CAVES_FLAG = 129;

	
	
	public void WorleyCaveGenerator()
//	public CubicCaveGenerator()
	{
		//TODO noise should probably be seeded with world seed
		worleyF1divF3.SetFrequency(0.016f);
		
		displacementNoisePerlin.SetNoiseType(FastNoise.NoiseType.Perlin);
		displacementNoisePerlin.SetFrequency(0.05f);
		
		maxCaveHeight = Configs.cavegen.maxCaveHeight;
		noiseCutoff = (float) Configs.cavegen.noiseCutoffValue;
		warpAmplifier = (float) Configs.cavegen.warpAmplifier;
		easeInDepth = (float) Configs.cavegen.easeInDepth;
		yCompression = (float) Configs.cavegen.verticalCompressionMultiplier;
		xzCompression = (float) Configs.cavegen.horizonalCompressionMultiplier;
		surfaceCutoff = (float) Configs.cavegen.surfaceCutoffValue;
		lavaDepth = Configs.cavegen.lavaDepth;
		additionalWaterChecks = Loader.isModLoaded("subterranaenwaters");
			
		
//		lava = BlockUtil.getStateFromString(Configs.cavegen.lavaBlock);
//		if(lava == null)
//		{
//			Main.LOGGER.error("Cannont find block " + Configs.cavegen.lavaBlock);
			lava = BLK_AIR;
//		}
		
		//try and grab other modded cave gens, like swiss cheese caves or Quark big caves
		//our replace cavegen event will ignore cave events when the original cave class passed in is a Worley cave
//		moddedCaveGen = net.minecraftforge.event.terraingen.TerrainGen.getModdedMapGen(this, net.minecraftforge.event.terraingen.InitMapGenEvent.EventType.CAVE);
//		if(moddedCaveGen != this)
//			replacementCaves = moddedCaveGen; //9118 - yeeting this shit til i can get it to work *normally*
//		else
//			replacementCaves = new MapGenCaves(); //default to vanilla caves if there are no other modded cave gens
	}
	
	private void debugValueAdjustments()
	{
		//lavaDepth = 10;
		//noiseCutoff = 0.18F;
		//warpAmplifier = 8.0F;
		//easeInDepth = 15;
		//xzCompression = 0.5f;
	}
	
//	@Override
//	public void generate(World worldIn, int x, int z, ChunkPrimer primer)
	public void generate2(World worldIn, CubePrimer primer, CubePos pos)
	{
		
		int x = pos.getX();
		int z = pos.getZ();
		int y = pos.getY();
		
		
		
		
//		int currentDim = worldIn.provider.getDimension();
//		this.world = worldIn;	//9118 - so "world" doesnt exist in the cubicchunks
								//"ICubicStructure" (which apparently corresponds to
								//mc's "MapGenBase" & I can't edit ICubicStructure
								//so I'll just remove these for now.
		//revert to vanilla cave generation for blacklisted dims    //9118 - no lol
//		for(int blacklistedDim: Configs.cavegen.blackListedDims)
//		{
//			if(currentDim == blacklistedDim)
//			{
//				this.replacementCaves.generate(worldIn, x, z, primer);
//				return;
//			} 
//		}
		
		debugValueAdjustments();
		boolean logTime = false; //TODO turn off
		long start = 0;
		if(logTime)
		{
			start = System.nanoTime();
		}
		
//		this.world = worldIn;
		this.generateWorleyCaves(worldIn, x, y, z, primer);
		
		
		if(logTime)
		{
			genTime[currentTimeIndex] = System.nanoTime() - start;//System.currentTimeMillis() - start;
			sum += genTime[currentTimeIndex];
			currentTimeIndex++;
			if (currentTimeIndex == genTime.length)
			{
				System.out.printf("%d chunk average: %.2f ms per chunk\n", numLogChunks, sum/((float)numLogChunks*1000000));
				sum = 0;
				currentTimeIndex = 0;
			}
		}
	}
	
	protected void generateWorleyCaves(World worldIn, int chunkX, int chunkY, int chunkZ, CubePrimer chunkPrimerIn)
    {
		int chunkMaxHeight = 256;
		int seaLevel = worldIn.getSeaLevel();
		float[][][] samples = sampleNoise(chunkX, chunkY, chunkZ, chunkMaxHeight+1, worldIn);
        float oneQuarter = 0.25F;
        float oneHalf = 0.5F;
        Biome currentBiome;
    	BlockPos realPos;
        //float cutoffAdjuster = 0F; //TODO one day, perlin adjustments to cutoff
    	
    	
		//each chunk divided into 4 subchunks along X axis
		for (int x = 0; x < 4; x++)
		{
			//each chunk divided into 4 subchunks along Z axis
			for (int z = 0; z < 4; z++)
			{
				int depth = 0;
				
				//don't bother checking all the other logic if there's nothing to dig in this column
				
				//each chunk divided into 128 subchunks along Y axis. Need lots of y sample points to not break things //9118 - *8 not 128
				for(int y = 7; y >= 0; y--)
				{
					//grab the 8 sample points needed from the noise values
					
					float x0y0z0 = samples[x][y][z];
					float x0y0z1 = samples[x][y][z+1];
					float x1y0z0 = samples[x+1][y][z];
					float x1y0z1 = samples[x+1][y][z+1];
					float x0y1z0 = samples[x][y+1][z];
					float x0y1z1 = samples[x][y+1][z+1];
					float x1y1z0 = samples[x+1][y+1][z];
					float x1y1z1 = samples[x+1][y+1][z+1];
					
                    //how much to increment noise along y value
                    //linear interpolation from start y and end y
					
					//9118 - i have no clue why it originally multiplied by -0.5. *0.5 works for me fine. *-0.5 just breaks everything (the last i used it anyway)
//                    float noiseStepY00 = (x0y1z0 - x0y0z0) * -oneHalf;
//                    float noiseStepY01 = (x0y1z1 - x0y0z1) * -oneHalf;
//                    float noiseStepY10 = (x1y1z0 - x1y0z0) * -oneHalf;
//                    float noiseStepY11 = (x1y1z1 - x1y0z1) * -oneHalf;
                    
                    float noiseStepY00 = (x0y1z0 - x0y0z0) * oneHalf;
                    float noiseStepY01 = (x0y1z1 - x0y0z1) * oneHalf;
                    float noiseStepY10 = (x1y1z0 - x1y0z0) * oneHalf;
                    float noiseStepY11 = (x1y1z1 - x1y0z1) * oneHalf;
                    
                    //noise values of 4 corners at y=0
                    float noiseStartX0 = x0y0z0;
                    float noiseStartX1 = x0y0z1;
                    float noiseEndX0 = x1y0z0;
                    float noiseEndX1 = x1y0z1;
                    
                    // loop through 2 blocks of the Y subchunk
                    for (int suby = 0; suby < 2; suby++)
                    {
                    	int localY = suby + y*2;
                        float noiseStartZ = noiseStartX0;
                        float noiseEndZ = noiseStartX1;
                        
                    	int realY = localY + chunkY*16;
                    	
                        //how much to increment X values, linear interpolation
                        float noiseStepX0 = (noiseEndX0 - noiseStartX0) * oneQuarter;
                        float noiseStepX1 = (noiseEndX1 - noiseStartX1) * oneQuarter;

                        // loop through 4 blocks of the X subchunk
                        for (int subx = 0; subx < 4; subx++)
                        {
                        	int localX = subx + x*4;
                        	int realX = localX + chunkX*16;
                        	
                        	//how much to increment Z values, linear interpolation
                            float noiseStepZ = ((noiseEndZ - noiseStartZ) * oneQuarter);
                            
                            //Y and X already interpolated, just need to interpolate final 4 Z block to get final noise value
                            float noiseVal = noiseStartZ;

                            // loop through 4 blocks of the Z subchunk
                            for (int subz = 0; subz < 4; subz++)
                            {
                            	int localZ = subz + z*4;
                            	int realZ = localZ + chunkZ*16;
                            	
                    			realPos = new BlockPos(realX, realY, realZ);
                            	
                            	currentBiome = null;
                            	
                            	if(depth == 0)
                            	{
                            		//only checks depth once per 4x4 subchunk
                            		if(subx == 0 && subz == 0)
                            		{
                            			IBlockState currentBlock = chunkPrimerIn.getBlockState(localX, localY, localZ);
	                            		currentBiome = worldIn.provider.getBiomeProvider().getBiome(realPos, Biomes.PLAINS);
	                            		
	                            		//use isDigable to skip leaves/wood getting counted as surface //TODO: 9118 - remove this after set max height.
	            						if(canReplaceBlock(currentBlock, BLK_AIR) || isBiomeBlock(chunkPrimerIn, realX, realZ, currentBlock, currentBiome))
	            						{
	            							depth++;
	            						}
                            		}
            						else
            						{
            							continue;
            						}
                            	}
                            	else if(subx == 0 && subz == 0)
                            	{
                            		//already hit surface, simply increment depth counter
                            		depth++;
                            	}

                            	float adjustedNoiseCutoff = noiseCutoff;// + cutoffAdjuster;
                            	
            					if (noiseVal > adjustedNoiseCutoff)
            					{
            						IBlockState localBlockState;
            						try {
            							localBlockState = chunkPrimerIn.getBlockState(localX, localY+1, localZ);
            						}
            						catch (Exception e) {
//            							logger.error(e+" "+localX+" "+localY+1+" "+localZ);
            							localBlockState = BLK_AIR; //TODO: 9118 - see if making this "STONE" improves things
            						}
            						
            						
            						IBlockState aboveBlock = (IBlockState) MoreObjects.firstNonNull(localBlockState, BLK_AIR);
	            						
        							IBlockState currentBlock = chunkPrimerIn.getBlockState(localX, localY, localZ);
            						
        							//TODO: 9118 - revert this if it crashes
//        							IBlockState currentBlock;
//                            		try {
//                            			currentBlock = chunkPrimerIn.getBlockState(localX, localY, localZ);
//                            		}
//                            		catch(Exception e) {
//                            			logger.error(e+" "+localX+" "+localY+" "+localZ);
//                            			currentBlock = STONE;
//                            		}
            						
            						if(currentBiome == null) {
        								currentBiome = worldIn.provider.getBiomeProvider().getBiome(realPos, Biomes.PLAINS);//world.getBiome(realPos);
            						}
            						
            						boolean foundTopBlock = isTopBlock(currentBlock, currentBiome);
            						digBlock(chunkPrimerIn, localX, localY, localZ, chunkX, chunkZ, foundTopBlock, currentBlock, aboveBlock, currentBiome);
            					}
                                
                                noiseVal += noiseStepZ;
                            }

                            noiseStartZ += noiseStepX0;
                            noiseEndZ += noiseStepX1;
                        }

                        noiseStartX0 += noiseStepY00;
                        noiseStartX1 += noiseStepY01;
                        noiseEndX0 += noiseStepY10;
                        noiseEndX1 += noiseStepY11;
                    }
				}
			}	
		}
    }
	
	public float[][][] sampleNoise(int chunkX, int chunkY, int chunkZ, int maxSurfaceHeight, World worldIn) 
	{
		int originalMaxHeight = 128;
		float[][][] noiseSamples = new float[5][10][5];
		float noise;
		for (int x = 0; x < 5; x++)
		{
			int realX = x*4 + chunkX*16;
			for (int z = 0; z < 5; z++)
			{
				int realZ = z*4 + chunkZ*16;
				for (int y = 8; y >= 0; y--)
				{
					int realY = y*2 + chunkY*16;
					if(realY > maxCaveHeight)
					{
						//if outside of valid cave range set noise value below normal minimum of -1.0
						noiseSamples[x][y][z] = -1.1F;
					}
					else
					{
						//Experiment making the cave system more chaotic the more you descend 
						///TODO might be too dramatic down at lava level
						///TODO 9118 - might need to divide the warpAmplifier by something.
						float dispAmp = (float) ((2.6 * sineNoise1d(y/256)) + 5);// = (float) (warpAmplifier * sineNoise1d(y));//(float) (warpAmplifier * ((originalMaxHeight-(realY/2))/(originalMaxHeight*0.85)));
						
						BlockPos realPos = new BlockPos(realX, realY, realZ);
						Biome currentBiome = worldIn.provider.getBiomeProvider().getBiome(realPos, Biomes.PLAINS);
						
						if(currentBiome == Biomes.EXTREME_HILLS || currentBiome == Biomes.EXTREME_HILLS_WITH_TREES)
						{
							dispAmp = (float) (8 * sineNoise1d(y));
						}
						else if(currentBiome == Biomes.MUTATED_EXTREME_HILLS || currentBiome == Biomes.MUTATED_EXTREME_HILLS_WITH_TREES)
						{
							dispAmp = (float) (Math.pow(8 * sineNoise1d(y), 2));
						}
						else if(currentBiome == Biomes.EXTREME_HILLS_EDGE)
						{
							dispAmp = (float) (4 * sineNoise1d(y));
						}
						else if(currentBiome == Biomes.PLAINS)
						{
							dispAmp = 0.1f;
						}
						
						
						
						/*
						else if(currentBiome == Biomes)
						{
							dispAmp = ;
						}
						*/
						
						
						float xDisp = 0f;
						float yDisp = 0f;
						float zDisp = 0f;
						
						xDisp = displacementNoisePerlin.GetNoise(realX, realZ)*dispAmp; 
						yDisp = displacementNoisePerlin.GetNoise(realX, realZ+67.0f)*dispAmp;
						zDisp = displacementNoisePerlin.GetNoise(realX, realZ+149.0f)*dispAmp;
						
						double caveSize = caveSizeAtY(realY);
						float hCompress = xzCompression/(float) caveSize;
						float vCompress = yCompression/(float) caveSize;
						
						float noiseCutoffMod = noiseCutoff;//Math.max((noiseCutoff+(realY/256)), noiseCutoff);
						
						
						//doubling the y frequency to get some more caves
						noise = worleyF1divF3.SingleCellular3Edge(realX*hCompress+xDisp, realY*vCompress+yDisp, realZ*hCompress+zDisp);
						noiseSamples[x][y][z] = noise;
						
						if (noise > noiseCutoffMod)//noiseCutoff)
						{
							//if noise is below cutoff, adjust values of neighbors
							//helps prevent caves fracturing during interpolation
							
							if(x > 0)
								noiseSamples[x-1][y][z] = (noise*0.2f) + (noiseSamples[x-1][y][z]*0.8f);
							if(z > 0)
								noiseSamples[x][y][z-1] = (noise*0.2f) + (noiseSamples[x][y][z-1]*0.8f);
							
							
							//more heavily adjust y above 'air block' noise values to give players more headroom
							if(y*2 < 16)
							{
								float noiseAbove = noiseSamples[x][y+1][z];
								if(noise > noiseAbove)
									noiseSamples[x][y+1][z] = (noise*0.8F) + (noiseAbove*0.2F);
								if(y*2 < 15)
								{
									float noiseTwoAbove = noiseSamples[x][y+2][z];
									if(noise > noiseTwoAbove)
										noiseSamples[x][y+2][z] = (noise*0.35F) + (noiseTwoAbove*0.65F);
								}
							}
						}
					}
				}
			}
		}
		return noiseSamples;
	}
	
	//returns true if block matches the top or filler block of the location biome
	private boolean isBiomeBlock(CubePrimer primer, int realX, int realZ, IBlockState state, Biome biome)
	{
		return state == biome.topBlock || state == biome.fillerBlock;
	}
	
	//returns true if block is fluid, trying to play nice with modded liquid
	private boolean isFluidBlock(IBlockState state)
	{
		Block blocky = state.getBlock();
		return blocky instanceof BlockLiquid || blocky instanceof IFluidBlock;
	}
	
	//Because it's private in MapGenCaves this is reimplemented
	//Determine if the block at the specified location is the top block for the biome, we take into account
    //Vanilla bugs to make sure that we generate the map the same way vanilla does.
    private boolean isTopBlock(IBlockState state, Biome biome)
    {
        //IBlockState state = data.getBlockState(x, y, z);
        return (isExceptionBiome(biome) ? state.getBlock() == Blocks.GRASS : state == biome.topBlock);
    }
    
    //Exception biomes to make sure we generate like vanilla
    private boolean isExceptionBiome(net.minecraft.world.biome.Biome biome)
    {
        if (biome == net.minecraft.init.Biomes.BEACH) return true;
        if (biome == net.minecraft.init.Biomes.DESERT) return true;
        return false;
    }

//    @Override //9118 - idk what the hell i'm doin.
    protected boolean canReplaceBlock(IBlockState state, IBlockState stateUp)
    {
        // Need to be able to replace not just vanilla stone + stuff
//        return (Configs.cavegen.allowReplaceMoreBlocks && state.getMaterial() == Material.ROCK) || super.canReplaceBlock(state, stateUp);
    	
        //----------------------
        if (Configs.cavegen.allowReplaceMoreBlocks && state.getMaterial() == Material.ROCK)
        {
        	return true;
        }
        //----------------------
        else if (state.getBlock() == Blocks.STONE)
        {
            return true;
        }
        else if (state.getBlock() == Blocks.DIRT)
        {
            return true;
        }
        else if (state.getBlock() == Blocks.GRASS)
        {
            return true;
        }
        else if (state.getBlock() == Blocks.HARDENED_CLAY)
        {
            return true;
        }
        else if (state.getBlock() == Blocks.STAINED_HARDENED_CLAY)
        {
            return true;
        }
        else if (state.getBlock() == Blocks.SANDSTONE)
        {
            return true;
        }
        else if (state.getBlock() == Blocks.RED_SANDSTONE)
        {
            return true;
        }
        else if (state.getBlock() == Blocks.MYCELIUM)
        {
            return true;
        }
        else if (state.getBlock() == Blocks.SNOW_LAYER)
        {
            return true;
        }
        else
        {
            return (state.getBlock() == Blocks.SAND || state.getBlock() == Blocks.GRAVEL) && state.getMaterial() != Material.WATER;
        }
        //----------------------
    }
    
    /**
     * Digs out the current block, default implementation removes stone, filler, and top block
     * Sets the block to lava if y is less then 10, and air other wise.
     * If setting to air, it also checks to see if we've broken the surface and if so
     * tries to make the floor the biome's top block
     *
     * @param data Block data array
     * @param index Pre-calculated index into block data
     * @param x local X position
     * @param y local Y position
     * @param z local Z position
     * @param chunkX Chunk X position
     * @param chunkZ Chunk Y position
     * @param foundTop True if we've encountered the biome's top block. Ideally if we've broken the surface.
     */
    protected void digBlock(CubePrimer data, int x, int y, int z, int chunkX, int chunkZ, boolean foundTop, IBlockState state, IBlockState up, Biome biome)
    {
//		logger.info("dig block");
    	
        IBlockState top = biome.topBlock;
        IBlockState filler = biome.fillerBlock;
        

        if (this.canReplaceBlock(state, up) || state.getBlock() == top.getBlock() || state.getBlock() == filler.getBlock())
        {
            data.setBlockState(x, y, z, BLK_AIR);
            
            if(y > 0) {
                if (foundTop && data.getBlockState(x, y - 1, z).getBlock() == filler.getBlock())
                {
                    data.setBlockState(x, y - 1, z, top);
                }
            }
            
            //replace floating sand with sandstone
            if(up == SAND)
            {
            	data.setBlockState(x, y+1, z, BLK_SANDSTONE);
            }
            else if(up == RED_SAND)
            {
            	data.setBlockState(x, y+1, z, BLK_RED_SANDSTONE);
            }
        }
    }
    //------------------------------------------------------------------------------------------------------------
    
    private double caveSizeAtY(int realY) {
    	double ypos = -realY;
    	double cave = 32; //9118 - this is 32 to try and reduce the slicing effect when going through a transition layer. it half worked. eh, good enough.
    	
    	//9118 - this finds the cave level at the current y. each power of 2 gets a long layer, the numbers between are grouped into "transition layers" which transition between the powers of 2 somewhat succesfully.
    	for(long y = 0; y < ypos; cave += 1){ //9118 - i would've used an equation to do this, but it turned out that that was difficult and a for loop is easy.
    		if(cave == 32 || isPowerOfTwo(cave/32)){
    			y += cave*2;
    		}
    		else{
    			y++;
    		}
    	}
    	
    	return cave/32;
    }
    
    private boolean isPowerOfTwo(double i)
    {
    	if(i % 1 != 0)
    	{
    		return false;
    	};
    	long j = (long) i;
        return (j != 0) && ((j & (j - 1)) == 0);
    }
    
    private double sineNoise1d(double x)
    {
    	double FactorE = -1.2;
    	double ScaleE = -1.7;
    	double FactorPi = 1.9;
    	double ScalePi = 0.7;
    	double Factor1 = -3.2;
    	double Scale1 = -1.3;
    	double FactorTotal = 0.3;
    	return FactorTotal * (Factor1 * Math.sin(Scale1 * x) + FactorE * Math.sin(ScaleE * Math.E * x) + FactorPi * Math.sin(ScalePi * Math.PI * x));
    }
    
    //------------------------------------------------------------------------------------------------------------
}
