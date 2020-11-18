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

import com.google.common.base.Preconditions;
import io.github.opencubicchunks.cubicchunks.api.worldgen.populator.CubicPopulatorList;
import io.github.opencubicchunks.cubicchunks.api.worldgen.populator.ICubicPopulator;
import io.github.opencubicchunks.cubicchunks.cubicgen.CustomCubicMod;
import io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.CustomGeneratorSettings;
import io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.populator.AnimalsPopulator;
import io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.populator.DefaultDecorator;
import io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.populator.PrePopulator;
import io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.populator.SurfaceSnowPopulator;
import mcp.MethodsReturnNonnullByDefault;
import net.minecraft.util.ResourceLocation;
import net.minecraft.world.biome.Biome;
import net.minecraftforge.fml.common.registry.ForgeRegistries;
import net.minecraftforge.registries.IForgeRegistry;
import net.minecraftforge.registries.IForgeRegistryEntry;
import net.minecraftforge.registries.RegistryBuilder;

import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import javax.annotation.ParametersAreNonnullByDefault;

//
@ParametersAreNonnullByDefault
@MethodsReturnNonnullByDefault
public final class CubicBiome extends IForgeRegistryEntry.Impl<CubicBiome> {

    public static IForgeRegistry<CubicBiome> REGISTRY;
    private static final Map<Biome, CubicBiome> biomeMapping = new IdentityHashMap<>();
    private static boolean isPostInit = false;

    private final Biome originalBiome;
    private final List<IBiomeBlockReplacerProvider> blockReplacers = new ArrayList<>();
    private Function<CustomGeneratorSettings, ICubicPopulator> decoratorProvider;

    public Iterable<IBiomeBlockReplacerProvider> getReplacerProviders() {
        return Collections.unmodifiableList(blockReplacers);
    }

    public ICubicPopulator getDecorator(CustomGeneratorSettings conf) {
        return decoratorProvider.apply(conf);
    }

    // INTERNAL USE ONLY
    public static void init() {
        REGISTRY = new RegistryBuilder<CubicBiome>()
                .setType(CubicBiome.class)
                .setIDRange(0, 256)
                .setName(new ResourceLocation(CustomCubicMod.MODID, "cubic_biome_registry"))
                .create();
    }

    public static void postInit() {
        if (isPostInit) {
            return;
        }
        isPostInit = true;

        boolean anyUnregistered = false;
        // make sure that all registered cubic biomes are for biomes that are actually registered
        for (CubicBiome cubicBiome : CubicBiome.REGISTRY) {
            Biome biome = cubicBiome.getBiome();
            biomeMapping.put(biome, cubicBiome);
            if (!ForgeRegistries.BIOMES.containsValue(biome)) {
                anyUnregistered = true;
                CustomCubicMod.LOGGER.error(
                        "Registered cubic chunks biome has unregistered biome {} (name={}, class={}) is not allowed",
                        biome.getRegistryName(), biome, biome.getBiomeName());
            }
        }
        if (anyUnregistered) {
            throw new IllegalStateException("Found one or more unregistered biomes with registered cubic chunks biomes");
        }

        for (Biome biome : ForgeRegistries.BIOMES) {
            if (!biomeMapping.containsKey(biome)) {
                CustomCubicMod.LOGGER
                        .warn("Biome {} not registered as cubic chunks biome, will use default unregistered biome instead", biome.getRegistryName());

                CubicBiome newBiome = CubicBiome
                        .createForBiome(biome)
                        .defaults()
                        .defaultDecorators()
                        .setRegistryName(new ResourceLocation(CustomCubicMod.MODID, "unregistered_" + biome.getRegistryName().getPath()))
                        .create();
                biomeMapping.put(biome, newBiome);
            }
        }
    }

    private CubicBiome(Builder builder) {
        this.originalBiome = builder.biome;
        this.blockReplacers.addAll(builder.blockReplacers);
        this.decoratorProvider = conf -> {
            CubicPopulatorList list = new CubicPopulatorList();
            builder.decorators.forEach(func -> list.add(func.apply(conf)));
            return list;
        };

        this.setRegistryName(builder.registryName);
    }

    public Biome getBiome() {
        return this.originalBiome;
    }

    @Override
    public String toString() {
        return this.getRegistryName().toString();
    }

    // equals and hashcode should only check the biome
    // the only cubic biome with null biome is the unregistered one
    @Override public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }

        CubicBiome that = (CubicBiome) o;

        return originalBiome != null ? originalBiome.equals(that.originalBiome) : that.originalBiome == null;

    }

    @Override public int hashCode() {
        return originalBiome != null ? originalBiome.hashCode() : 0;
    }

    public static CubicBiome getCubic(Biome vanillaBiome) {
        return biomeMapping.get(vanillaBiome);
    }

    public static IBiomeBlockReplacerProvider terrainShapeReplacer() {
        return TerrainShapeReplacer.provider();
    }

    public static IBiomeBlockReplacerProvider oceanWaterReplacer() {
        return OceanWaterReplacer.provider();
    }

    public static IBiomeBlockReplacerProvider surfaceDefaultReplacer() {
        return SurfaceDefaultReplacer.provider();
    }

    public static CubicBiome.Builder createForBiome(Biome biome) {
        return new Builder(biome);
    }

    public static class Builder {

        private final Biome biome;
        private List<IBiomeBlockReplacerProvider> blockReplacers = new ArrayList<>();
        private ResourceLocation registryName;
        private final List<Function<CustomGeneratorSettings, ICubicPopulator>> decorators = new ArrayList<>();

        public Builder(Biome biome) {
            this.biome = biome;
        }

        public Builder defaults() {
            return addDefaultBlockReplacers()
                    .defaultDecorators();
        }

        public Builder addDefaultBlockReplacers() {
            return addBlockReplacer(terrainShapeReplacer())
                    .addBlockReplacer(surfaceDefaultReplacer())
                    .addBlockReplacer(oceanWaterReplacer());
        }

        public Builder addBlockReplacer(IBiomeBlockReplacerProvider provider) {
            Preconditions.checkNotNull(provider);
            this.blockReplacers.add(provider);
            return this;
        }

        public Builder defaultDecorators() {
            this.decoratorProvider(PrePopulator::new);
            this.decoratorProvider(DefaultDecorator.Ores::new);
            this.decoratorProvider(DefaultDecorator::new);
            return this;
        }

        public Builder defaultPostDecorators() {
            this.decoratorProvider(c -> new AnimalsPopulator());
            this.decoratorProvider(c -> new SurfaceSnowPopulator());
            return this;
        }

        public Builder decoratorProvider(Function<CustomGeneratorSettings, ICubicPopulator> decorator) {
            this.decorators.add(decorator);
            return this;
        }

        public Builder decorator(ICubicPopulator populator) {
            return decoratorProvider(c -> populator);
        }

        public Builder setRegistryName(ResourceLocation registryName) {
            this.registryName = registryName;
            return this;
        }

        public Builder setRegistryName(String modid, String resourcePath) {
            return this.setRegistryName(new ResourceLocation(modid, resourcePath));
        }

        public CubicBiome create() {
            if (this.registryName == null) {
                this.registryName = biome.getRegistryName();
            }
            return new CubicBiome(this);
        }
    }
}
