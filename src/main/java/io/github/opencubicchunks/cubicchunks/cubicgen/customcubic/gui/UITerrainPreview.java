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
package io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.gui;

import static com.flowpowered.noise.Noise.gradientNoise3D;
import static io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.MalisisGuiUtils.malisisText;

import com.flowpowered.noise.NoiseQuality;
import com.flowpowered.noise.Utils;
import io.github.opencubicchunks.cubicchunks.api.util.MathUtil;
import io.github.opencubicchunks.cubicchunks.cubicgen.ConversionUtils;
import io.github.opencubicchunks.cubicchunks.cubicgen.CustomCubicMod;
import io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.component.UIShaderComponent;
import io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.render.DynamicTexture;
import io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.CustomGeneratorSettings;
import net.malisis.core.client.gui.ClipArea;
import net.malisis.core.client.gui.GuiRenderer;
import net.malisis.core.client.gui.component.IClipable;
import net.malisis.core.client.gui.element.SimpleGuiShape;
import net.malisis.core.renderer.animation.Animation;
import net.malisis.core.renderer.animation.transformation.ITransformable;
import net.malisis.core.renderer.font.FontOptions;
import net.malisis.core.renderer.font.MalisisFont;
import net.malisis.core.util.MouseButton;
import net.minecraft.client.Minecraft;
import net.minecraft.client.renderer.GlStateManager;
import net.minecraft.client.shader.ShaderManager;
import net.minecraft.util.EnumFacing;
import net.minecraft.util.math.MathHelper;
import net.minecraft.world.biome.Biome;
import net.minecraftforge.fml.common.registry.ForgeRegistries;
import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector2f;
import org.lwjgl.util.vector.Vector3f;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.List;
import java.util.Random;
import java.util.function.DoubleSupplier;

public class UITerrainPreview extends UIShaderComponent<UITerrainPreview> implements ITransformable.Scale, IClipable {

    private static ShaderManager shader;
    private static final BufferedImage perlinTexture;

    static {
        int texSize = 32;
        int samplesPerPreiod = 8;
        float freq = 1.0f / samplesPerPreiod;

        perlinTexture = new BufferedImage(texSize*samplesPerPreiod, texSize*samplesPerPreiod, BufferedImage.TYPE_INT_ARGB);

        // replicate seed selection logic in CustomTerrainGenerator
        Random rnd = new Random(123456);
        long rawSeedSel = rnd.nextLong();
        long rawSeedLow = rnd.nextLong();
        long rawSeedHigh = rnd.nextLong();
        long rawSeedDepth = rnd.nextLong();
        int seedSel = (int) ((rawSeedSel & 0xFFFFFFFF) ^ (rawSeedSel >>> 32));
        int seedLow = (int) ((rawSeedLow & 0xFFFFFFFF) ^ (rawSeedLow >>> 32));
        int seedHigh = (int) ((rawSeedHigh & 0xFFFFFFFF) ^ (rawSeedHigh >>> 32));
        int seedDepth = (int) ((rawSeedDepth & 0xFFFFFFFF) ^ (rawSeedDepth >>> 32));

        for (int x = 0; x < perlinTexture.getWidth(); x++) {
            for (int y = 0; y < perlinTexture.getHeight(); y++) {
                float sel = (float) gradientCoherentNoise3DTileable(
                        x * freq, y * freq, 0,
                        seedSel, NoiseQuality.BEST, texSize - 1);
                float low = (float) gradientCoherentNoise3DTileable(
                        x * freq, y * freq, 0,
                        seedLow, NoiseQuality.BEST, texSize - 1);
                float high = (float) gradientCoherentNoise3DTileable(
                        x * freq, y * freq, 0,
                        seedHigh, NoiseQuality.BEST, texSize - 1);
                float depth = (float) gradientCoherentNoise3DTileable(
                        x * freq, y * freq, 0,
                        seedDepth, NoiseQuality.BEST, texSize - 1);

                int r = MathUtil.to8bitComponent(sel);
                int g = MathUtil.to8bitComponent(low);
                int b = MathUtil.to8bitComponent(high);
                int a = MathUtil.to8bitComponent(depth);
                int col = MathUtil.packColorARGB(r, g, b, a);
                perlinTexture.setRGB(x, y, col);
            }
        }
    }

    private Matrix4f previewTransform = new Matrix4f();
    private Animation<Scale> zoomAnim;
    private float biomeScale = 0.01f, biomeOffset = 0;
    private EnumFacing.Axis shownAxis;
    private TerrainPreviewDataAccess dataAccess;

    public UITerrainPreview(CustomCubicGui gui, TerrainPreviewDataAccess dataAccess) {
        super(gui, getShaderAndInitGL(gui));
        this.dataAccess = dataAccess;
        this.icon.flip(false, true);
        this.previewTransform.m31 = 64; // set y offset
    }

    private static ShaderManager getShaderAndInitGL(CustomCubicGui gui) {
        if (shader == null) {
            shader = createShader(gui);
        }
        return shader;
    }

    public void setBiomeScale(float biomeScale) {
        this.biomeScale = biomeScale;
    }

    public void setBiomeOffset(float biomeOffset) {
        this.biomeOffset = biomeOffset;
    }

    @Override public boolean onScrollWheel(int x, int y, int delta) {
        float factor = (float) Math.sqrt(2);
        if (delta > 0) {
            factor = 1 / factor;
        }
        DynamicZoomTransform transform;
        if (zoomAnim != null && !zoomAnim.isFinished()) {
            transform = ((DynamicZoomTransform) zoomAnim.getTransformation());
            getGui().stopAnimation(zoomAnim);
        } else {
            transform = new DynamicZoomTransform();
            transform.setTarget(getZoom());
        }
        transform.setStart(getZoom());
        transform.scaleTarget(factor);
        transform.forTicks(5).delay(0).offset(x - screenX() - getWidth() / 2, y - screenY() - getHeight() / 2, 0);
        this.zoomAnim = new Animation<>(this, transform);
        getGui().animate(this.zoomAnim);
        return true;
    }

    @Override public void scale(float x, float y, float z, float offsetX, float offsetY, float offsetZ) {
        //System.out.println(offsetX + ", " + offsetY);
        previewTransform.translate(new Vector2f(offsetX, -offsetY))
                .scale(new Vector3f(x / previewTransform.m00, y / previewTransform.m11, z / previewTransform.m22))
                .translate(new Vector2f(-offsetX, offsetY));
    }

    @Override public boolean onDrag(int lastX, int lastY, int x, int y, MouseButton button) {
        previewTransform = previewTransform.translate(new Vector2f(lastX - x, y - lastY));
        return true;
    }


    @Override protected void shaderDraw(GuiRenderer guiRenderer, int mouseX, int mouseY, float partialTicks) {
        //previewTransform.m30 = (float) Math.sin((System.currentTimeMillis()%10000)*0.001f)*20;
        this.icon.setUVs(-getWidth() / 2, -getHeight() / 2, getWidth() / 2, getHeight() / 2);

        final float biomeCount = ForgeRegistries.BIOMES.getValuesCollection().size();

        shader.getShaderUniformOrDefault("previewTransform").set(getZoomOffsetMatrix());
        shader.getShaderUniformOrDefault("biomeCoordScaleAndOffset").set(1 / (biomeCount * biomeScale), biomeOffset / biomeCount);
        shader.getShaderUniformOrDefault("waterLevel").set((float) dataAccess.waterLevel.getAsDouble());
        shader.getShaderUniformOrDefault("biomeCount").set(ForgeRegistries.BIOMES.getValuesCollection().size());

        shader.getShaderUniformOrDefault("heightVariationFactor").set((float) dataAccess.heightVariationFactor.getAsDouble());
        shader.getShaderUniformOrDefault("heightVariationSpecial").set((float) dataAccess.specialHeightVariationFactorBelowAverageY.getAsDouble());
        shader.getShaderUniformOrDefault("heightVariationOffset").set((float) dataAccess.heightVariationOffset.getAsDouble());
        shader.getShaderUniformOrDefault("heightFactor").set((float) dataAccess.heightFactor.getAsDouble());
        shader.getShaderUniformOrDefault("heightOffset").set((float) dataAccess.heightOffset.getAsDouble());

        shader.getShaderUniformOrDefault("depthFactor").set((float) dataAccess.depthNoiseFactor.getAsDouble());
        shader.getShaderUniformOrDefault("depthOffset").set((float) dataAccess.depthNoiseOffset.getAsDouble());
        shader.getShaderUniformOrDefault("depthFreq").set(shownAxis == EnumFacing.Axis.X ? (float) dataAccess.depthNoiseFrequencyX.getAsDouble() :
                (float) dataAccess.depthNoiseFrequencyZ.getAsDouble());
        // this set method should be named setSafeInts
        shader.getShaderUniformOrDefault("depthOctaves").set((int) dataAccess.depthNoiseOctaves.getAsDouble(), 0, 0, 0);

        shader.getShaderUniformOrDefault("selectorFactor").set((float) dataAccess.selectorNoiseFactor.getAsDouble());
        shader.getShaderUniformOrDefault("selectorOffset").set((float) dataAccess.selectorNoiseOffset.getAsDouble());
        shader.getShaderUniformOrDefault("selectorFreq").set(
                shownAxis == EnumFacing.Axis.X ? (float) dataAccess.selectorNoiseFrequencyX.getAsDouble() :
                        (float) dataAccess.selectorNoiseFrequencyZ.getAsDouble(),
                (float) dataAccess.selectorNoiseFrequencyY.getAsDouble());
        shader.getShaderUniformOrDefault("selectorOctaves").set((int) dataAccess.selectorNoiseOctaves.getAsDouble(), 0, 0, 0);

        shader.getShaderUniformOrDefault("lowFactor").set((float) dataAccess.lowNoiseFactor.getAsDouble());
        shader.getShaderUniformOrDefault("lowOffset").set((float) dataAccess.lowNoiseOffset.getAsDouble());
        shader.getShaderUniformOrDefault("lowFreq").set(
                shownAxis == EnumFacing.Axis.X ? (float) dataAccess.lowNoiseFrequencyX.getAsDouble() :
                        (float) dataAccess.lowNoiseFrequencyZ.getAsDouble(),
                (float) dataAccess.lowNoiseFrequencyY.getAsDouble());
        shader.getShaderUniformOrDefault("lowOctaves").set((int) dataAccess.lowNoiseOctaves.getAsDouble(), 0, 0, 0);

        shader.getShaderUniformOrDefault("highFactor").set((float) dataAccess.highNoiseFactor.getAsDouble());
        shader.getShaderUniformOrDefault("highOffset").set((float) dataAccess.highNoiseOffset.getAsDouble());
        shader.getShaderUniformOrDefault("highFreq").set(
                shownAxis == EnumFacing.Axis.X ? (float) dataAccess.highNoiseFrequencyX.getAsDouble() :
                        (float) dataAccess.highNoiseFrequencyZ.getAsDouble(),
                (float) dataAccess.highNoiseFrequencyY.getAsDouble());
        shader.getShaderUniformOrDefault("highOctaves").set((int) dataAccess.highNoiseOctaves.getAsDouble(), 0, 0, 0);

        super.shaderDraw(guiRenderer, mouseX, mouseY, partialTicks);
    }

    @Override protected void postShaderDraw(GuiRenderer guiRenderer, int mouseX, int mouseY, float partialTicks) {
        if (!this.isEnabled()) {
            FontOptions fo = FontOptions.builder().color(0xFFFFFF).shadow().build();
            String text = malisisText("preview_disabled");
            float textWidth = MalisisFont.minecraftFont.getStringWidth(text, fo);
            float textHeight = MalisisFont.minecraftFont.getStringHeight();
            guiRenderer.drawText(MalisisFont.minecraftFont, text, getWidth() / 2 - textWidth / 2, getHeight() / 2 - textHeight / 2, 0, fo);
            return;
        }
        GlStateManager.enableTexture2D();
        drawXScale();
        drawYScale();
        drawBiomeNames();
        renderer.next();// required to workaround
    }


    private void drawBiomeNames() {

        int biomeCount = ForgeRegistries.BIOMES.getValues().size();
        // TODO: attempt explaion why these work, found by trial and error
        float biomeWidthMCPixels = (biomeScale) / (getZoomX());
        float offsetBiomeIdFromCenter = getOffsetX() / biomeScale + biomeOffset;
        if (biomeWidthMCPixels < 15) {
            return;
        }
        float offsetIdFromLeft = offsetBiomeIdFromCenter - getWidth() / 2 / biomeWidthMCPixels;
        if (Float.isInfinite(biomeWidthMCPixels)) {
            // make it only render the biome name in the top left corner
            biomeWidthMCPixels = getWidth() * 10;
            offsetIdFromLeft = biomeOffset + 0.001f;
        }
        // how many pixels to the left the first biome starts?
        float biomeStartOffsetPixels = (-(float) MathHelper.frac(offsetIdFromLeft)) * biomeWidthMCPixels;

        float end = getWidth();

        FontOptions fo = new FontOptions.FontOptionsBuilder().color(0xFFFFFF).shadow(true).build();
        MalisisFont font = MalisisFont.minecraftFont;

        List<Biome> biomes = ForgeRegistries.BIOMES.getValues();
        for (float x = biomeStartOffsetPixels; x <= end; x += biomeWidthMCPixels) {
            int idx = Math.floorMod(Math.round(x / biomeWidthMCPixels + offsetIdFromLeft), biomeCount);

            // -1 for the green line, another one so it doesn't touch the line
            float maxWidth = biomeWidthMCPixels - 2;
            float displayX = x;
            if (displayX < 0) {
                maxWidth += displayX;
                displayX = 0;
            }
            if (maxWidth < 10) {
                continue;
            }
            String biomeName = biomes.get(idx).getBiomeName();
            if (font.getStringWidth(biomeName, fo) > maxWidth) {
                while (font.getStringWidth(biomeName + "...", fo) > maxWidth) {
                    biomeName = biomeName.substring(0, biomeName.length() - 1);
                }
                biomeName += "...";
            }
            renderer.drawText(MalisisFont.minecraftFont, biomeName, (int) displayX + 2, 0, 0, fo);
        }
        renderer.next();

        GlStateManager.disableTexture2D();
        SimpleGuiShape shape = new SimpleGuiShape();
        shape.setSize(1, getHeight());

        for (float x = biomeStartOffsetPixels; x <= end; x += biomeWidthMCPixels) {
            shape.storeState();
            shape.setPosition(MathHelper.floor(x), 0);
            rp.setColor(0x77FF77);
            rp.alpha.set(0x80);
            renderer.drawShape(shape, rp);

            shape.resetState();
        }
        renderer.next();
        GlStateManager.enableTexture2D();
    }

    private void drawXScale() {
        if (getWidth() <= 0 || getHeight() <= 0) {
            return;
        }
        float blockLeft = posToX(0);
        float blockRight = posToX(getWidth());

        float maxVal = Math.max(Math.abs(blockLeft), Math.abs(blockRight));
        int digits = MathHelper.ceil(Math.log(Math.max(maxVal, 1)) / Math.log(10)) + ((blockLeft < 0 || blockRight < 0) ? 1 : 0);
        if (digits <= 0) {
            return;
        }

        int increment = getIncrement(blockLeft, blockRight, getWidth() / (7 * digits));
        if (increment < 0 || maxVal + increment > Integer.MAX_VALUE) {
            return;// TODO: handle integer overflow for increment
        }
        int start = MathHelper.roundUp(MathHelper.floor(blockLeft), increment);

        FontOptions fo = new FontOptions.FontOptionsBuilder().color(0xFFFFFF).shadow(true).build();
        for (int x = start; x <= blockRight; x += increment) {
            int pos = (int) xToPos(x);
            int strWidth = (int) (MalisisFont.minecraftFont.getStringWidth(String.valueOf(Math.abs(x)), fo) / 2 +
                    (x < 0 ? MalisisFont.minecraftFont.getStringWidth("-", fo) : 0));
            int strPos = pos - strWidth + 1;
            renderer.drawText(MalisisFont.minecraftFont, String.valueOf(x), strPos, getHeight() - 10, 0, fo);
        }

        renderer.next();
        GlStateManager.disableTexture2D();
        SimpleGuiShape shape = new SimpleGuiShape();
        shape.setSize(1, 2);
        for (int x = start; x <= blockRight; x += increment) {
            int pos = (int) xToPos(x);
            int strWidth = (int) (MalisisFont.minecraftFont.getStringWidth(String.valueOf(x), fo) / 2);
            shape.storeState();
            shape.setPosition(pos, getHeight() - 1);
            renderer.drawShape(shape, rp);

            shape.resetState();
        }
        renderer.next();
        GlStateManager.enableTexture2D();
    }

    private void drawYScale() {
        float blockBottom = posToY(getHeight());// bottom -> getHeight()
        float blockTop = posToY(0);

        int increment = getIncrement(blockBottom, blockTop, getHeight() / 11);
        if (increment < 0 || Math.max(Math.abs(blockBottom) + increment, Math.abs(blockTop) + increment) > Integer.MAX_VALUE) {
            return;// TODO: handle integer overflow for increment
        }
        int start = MathHelper.roundUp(MathHelper.floor(blockBottom), increment);

        FontOptions fo = new FontOptions.FontOptionsBuilder().color(0xFFFFFF).shadow(true).build();
        for (int y = start; y <= blockTop; y += increment) {
            int pos = (int) yToPos(y);
            if (pos < 15 || pos > getHeight() - 15) {
                continue;
            }
            int strHeight = (int) (MalisisFont.minecraftFont.getStringHeight() / 2);
            // use the "-" character as graph mark
            renderer.drawText(MalisisFont.minecraftFont, "- " + y, 0, pos - strHeight, 0, fo);
        }

    }


    private float getOffsetX() {
        return previewTransform.m30;
    }

    private float getOffsetY() {
        return previewTransform.m31;
    }

    // TODO: don't use this method
    private float getZoom() {
        return previewTransform.m00;
    }

    private float getZoomX() {
        return previewTransform.m00;
    }

    private float getZoomY() {
        return previewTransform.m11;
    }

    private float posToY(float pos) {
        return getOffsetY() + getZoomY() * (-pos + getHeight() / 2);
    }

    private float yToPos(float y) {
        return -(y - getOffsetY()) / getZoomY() + getHeight() / 2;
    }

    private float posToX(float pos) {
        return getOffsetX() + getZoomX() * (pos - getWidth() / 2);
    }

    private float xToPos(float y) {
        return (y - getOffsetX()) / getZoomX() + getWidth() / 2;
    }

    private int getIncrement(float start, float end, int maxAmount) {
        float totalSize = Math.abs(end - start);

        int curr = 1;
        while (curr < totalSize / maxAmount) {
            long n = curr;
            if (MathUtil.isPowerOfN(curr, 10)) {
                n *= 2;
            } else if (MathUtil.isPowerOfN(curr / 2, 10)) {
                n /= 2;
                n *= 5;
            } else {
                assert MathUtil.isPowerOfN(curr / 5, 10); // 5*powerOf10
                n *= 2;
            }
            if (n != (int) n) {
                return -1; // integer overflow, show just the beginning and the end
            }
            curr = (int) n;
        }

        return curr;
    }

    private Matrix4f getZoomOffsetMatrix() {
        return previewTransform.transpose(new Matrix4f());
    }

    private static ShaderManager createShader(CustomCubicGui gui) {
        // Note: the actual resource location name used will be "shaders/program/" + resourceName + ".json"
        ShaderManager shader = null;
        try {
            shader = new ShaderManager(Minecraft.getMinecraft().getResourceManager(), CustomCubicMod.MODID + ":custom-cubic-preview");
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        shader.addSamplerTexture("perlin", generateNoiseTexture());
        shader.addSamplerTexture("biomes", generateBiomesTexture());
        return shader;
    }

    private static DynamicTexture generateNoiseTexture() {
        DynamicTexture img = new DynamicTexture(perlinTexture);
        img.loadTexture(null);
        return img;
    }

    private static DynamicTexture generateBiomesTexture() {
        int count = ForgeRegistries.BIOMES.getValues().size();
        BufferedImage data = new BufferedImage(count, 1, BufferedImage.TYPE_INT_ARGB);
        Biome[] biomes = ForgeRegistries.BIOMES.getValues().toArray(new Biome[0]);

        for (int x = 0; x < count; x++) {
            Biome biome = biomes[x];

            float h = ConversionUtils.biomeHeightVanilla(biome.getBaseHeight());
            float hv = ConversionUtils.biomeHeightVariationVanilla(biome.getHeightVariation());

            // r = height integer part (-128 to 127)
            // g = height fractional part
            // b = height variation integer part (-128 to 127)
            // a = height variation fractional part

            float hFrac = (float) MathHelper.frac(h);
            float hInt = MathHelper.floor(h);
            float hvFrac = (float) MathHelper.frac(hv);
            float hvInt = MathHelper.floor(hv);

            int r = MathHelper.clamp(Math.round(hInt + 128), 0, 255);
            int g = MathUtil.to8bitComponent(hFrac);

            int b = MathHelper.clamp(Math.round(hvInt + 128), 0, 255);
            int a = MathUtil.to8bitComponent(hvFrac);

            data.setRGB(x, 0, MathUtil.packColorARGB(r, g, b, a));
        }

        DynamicTexture obj = new DynamicTexture(data);
        obj.loadTexture(null);
        return obj;
    }


    private static double gradientCoherentNoise3DTileable(double x, double y, double z, int seed, NoiseQuality quality, int mask) {

        // Create a unit-length cube aligned along an integer boundary.  This cube
        // surrounds the input point.

        int x0 = ((x >= 0.0) ? (int) x : (int) x - 1);
        int x1 = x0 + 1;

        int y0 = ((y >= 0.0) ? (int) y : (int) y - 1);
        int y1 = y0 + 1;

        int z0 = ((z >= 0.0) ? (int) z : (int) z - 1);
        int z1 = z0 + 1;

        double fx = x - x0, fy = y - y0, fz = z - z0;
        // Map the difference between the coordinates of the input value and the
        // coordinates of the cube's outer-lower-left vertex onto an S-curve.
        double xs, ys, zs;
        if (quality == NoiseQuality.FAST) {
            xs = fx;
            ys = fy;
            zs = fz;
        } else if (quality == NoiseQuality.STANDARD) {
            xs = Utils.sCurve3(fx);
            ys = Utils.sCurve3(fy);
            zs = Utils.sCurve3(fz);
        } else {

            xs = Utils.sCurve5(fx);
            ys = Utils.sCurve5(fy);
            zs = Utils.sCurve5(fz);
        }

        x0 &= mask;
        x1 &= mask;
        y0 &= mask;
        y1 &= mask;
        z0 &= mask;
        z1 &= mask;

        // Now calculate the noise values at each vertex of the cube.  To generate
        // the coherent-noise value at the input point, interpolate these eight
        // noise values using the S-curve value as the interpolant (trilinear
        // interpolation.)
        double n0, n1, ix0, ix1, iy0, iy1;
        n0 = gradientNoise3D(x0 + fx, y0 + fy, z0 + fz, x0, y0, z0, seed);
        n1 = gradientNoise3D(x1 + fx - 1, y0 + fy, z0 + fz, x1, y0, z0, seed);
        ix0 = Utils.linearInterp(n0, n1, xs);

        n0 = gradientNoise3D(x0 + fx, y1 + fy - 1, z0 + fz, x0, y1, z0, seed);
        n1 = gradientNoise3D(x1 + fx - 1, y1 + fy - 1, z0 + fz, x1, y1, z0, seed);
        ix1 = Utils.linearInterp(n0, n1, xs);
        iy0 = Utils.linearInterp(ix0, ix1, ys);
        n0 = gradientNoise3D(x0 + fx, y0 + fy, z1 + fz - 1, x0, y0, z1, seed);
        n1 = gradientNoise3D(x1 + fx - 1, y0 + fy, z1 + fz - 1, x1, y0, z1, seed);
        ix0 = Utils.linearInterp(n0, n1, xs);
        n0 = gradientNoise3D(x0 + fx, y1 + fy - 1, z1 + fz - 1, x0, y1, z1, seed);
        n1 = gradientNoise3D(x1 + fx - 1, y1 + fy - 1, z1 + fz - 1, x1, y1, z1, seed);
        ix1 = Utils.linearInterp(n0, n1, xs);
        iy1 = Utils.linearInterp(ix0, ix1, ys);
        return Utils.linearInterp(iy0, iy1, zs);
    }

    @Override public ClipArea getClipArea() {
        return new ClipArea(this);
    }

    @Override public void setClipContent(boolean clip) {
    }

    @Override public boolean shouldClipContent() {
        return true;
    }

    public void setShownAxis(EnumFacing.Axis shownAxis) {
        this.shownAxis = shownAxis;
    }

    public EnumFacing.Axis getShownAxis() {
        return shownAxis;
    }

    public static class TerrainPreviewDataAccess {

        DoubleSupplier waterLevel;
        DoubleSupplier heightVariationFactor;
        DoubleSupplier specialHeightVariationFactorBelowAverageY;
        DoubleSupplier heightVariationOffset;
        DoubleSupplier heightFactor;
        DoubleSupplier heightOffset;
        DoubleSupplier depthNoiseFactor;
        DoubleSupplier depthNoiseOffset;
        DoubleSupplier depthNoiseFrequencyX;
        DoubleSupplier depthNoiseFrequencyZ;
        DoubleSupplier depthNoiseOctaves;
        DoubleSupplier selectorNoiseFactor;
        DoubleSupplier selectorNoiseOffset;
        DoubleSupplier selectorNoiseFrequencyX;
        DoubleSupplier selectorNoiseFrequencyZ;
        DoubleSupplier selectorNoiseFrequencyY;
        DoubleSupplier selectorNoiseOctaves;
        DoubleSupplier lowNoiseFactor;
        DoubleSupplier lowNoiseOffset;
        DoubleSupplier lowNoiseFrequencyX;
        DoubleSupplier lowNoiseFrequencyZ;
        DoubleSupplier lowNoiseFrequencyY;
        DoubleSupplier lowNoiseOctaves;
        DoubleSupplier highNoiseFactor;
        DoubleSupplier highNoiseOffset;
        DoubleSupplier highNoiseFrequencyX;
        DoubleSupplier highNoiseFrequencyZ;
        DoubleSupplier highNoiseFrequencyY;
        DoubleSupplier highNoiseOctaves;

        public TerrainPreviewDataAccess setWaterLevel(DoubleSupplier value) {
            this.waterLevel = value;
            return this;
        }

        public TerrainPreviewDataAccess setHeightVariationFactor(DoubleSupplier value) {
            this.heightVariationFactor = value;
            return this;
        }

        public TerrainPreviewDataAccess setSpecialHeightVariationFactorBelowAverageY(DoubleSupplier value) {
            this.specialHeightVariationFactorBelowAverageY = value;
            return this;
        }

        public TerrainPreviewDataAccess setHeightVariationOffset(DoubleSupplier value) {
            this.heightVariationOffset = value;
            return this;
        }

        public TerrainPreviewDataAccess setHeightFactor(DoubleSupplier value) {
            this.heightFactor = value;
            return this;
        }

        public TerrainPreviewDataAccess setHeightOffset(DoubleSupplier value) {
            this.heightOffset = value;
            return this;
        }

        public TerrainPreviewDataAccess setDepthNoiseFactor(DoubleSupplier value) {
            this.depthNoiseFactor = value;
            return this;
        }

        public TerrainPreviewDataAccess setDepthNoiseOffset(DoubleSupplier value) {
            this.depthNoiseOffset = value;
            return this;
        }

        public TerrainPreviewDataAccess setDepthNoiseFrequencyX(DoubleSupplier value) {
            this.depthNoiseFrequencyX = value;
            return this;
        }

        public TerrainPreviewDataAccess setDepthNoiseFrequencyZ(DoubleSupplier value) {
            this.depthNoiseFrequencyZ = value;
            return this;
        }

        public TerrainPreviewDataAccess setDepthNoiseOctaves(DoubleSupplier value) {
            this.depthNoiseOctaves = value;
            return this;
        }

        public TerrainPreviewDataAccess setSelectorNoiseFactor(DoubleSupplier value) {
            this.selectorNoiseFactor = value;
            return this;
        }

        public TerrainPreviewDataAccess setSelectorNoiseOffset(DoubleSupplier value) {
            this.selectorNoiseOffset = value;
            return this;
        }

        public TerrainPreviewDataAccess setSelectorNoiseFrequencyX(DoubleSupplier value) {
            this.selectorNoiseFrequencyX = value;
            return this;
        }

        public TerrainPreviewDataAccess setSelectorNoiseFrequencyZ(DoubleSupplier value) {
            this.selectorNoiseFrequencyZ = value;
            return this;
        }

        public TerrainPreviewDataAccess setSelectorNoiseFrequencyY(DoubleSupplier value) {
            this.selectorNoiseFrequencyY = value;
            return this;
        }

        public TerrainPreviewDataAccess setSelectorNoiseOctaves(DoubleSupplier value) {
            this.selectorNoiseOctaves = value;
            return this;
        }

        public TerrainPreviewDataAccess setLowNoiseFactor(DoubleSupplier value) {
            this.lowNoiseFactor = value;
            return this;
        }

        public TerrainPreviewDataAccess setLowNoiseOffset(DoubleSupplier value) {
            this.lowNoiseOffset = value;
            return this;
        }

        public TerrainPreviewDataAccess setLowNoiseFrequencyX(DoubleSupplier value) {
            this.lowNoiseFrequencyX = value;
            return this;
        }

        public TerrainPreviewDataAccess setLowNoiseFrequencyZ(DoubleSupplier value) {
            this.lowNoiseFrequencyZ = value;
            return this;
        }

        public TerrainPreviewDataAccess setLowNoiseFrequencyY(DoubleSupplier value) {
            this.lowNoiseFrequencyY = value;
            return this;
        }

        public TerrainPreviewDataAccess setLowNoiseOctaves(DoubleSupplier value) {
            this.lowNoiseOctaves = value;
            return this;
        }

        public TerrainPreviewDataAccess setHighNoiseFactor(DoubleSupplier value) {
            this.highNoiseFactor = value;
            return this;
        }

        public TerrainPreviewDataAccess setHighNoiseOffset(DoubleSupplier value) {
            this.highNoiseOffset = value;
            return this;
        }

        public TerrainPreviewDataAccess setHighNoiseFrequencyX(DoubleSupplier value) {
            this.highNoiseFrequencyX = value;
            return this;
        }

        public TerrainPreviewDataAccess setHighNoiseFrequencyZ(DoubleSupplier value) {
            this.highNoiseFrequencyZ = value;
            return this;
        }

        public TerrainPreviewDataAccess setHighNoiseFrequencyY(DoubleSupplier value) {
            this.highNoiseFrequencyY = value;
            return this;
        }

        public TerrainPreviewDataAccess setHighNoiseOctaves(DoubleSupplier value) {
            this.highNoiseOctaves = value;
            return this;
        }
    }
}
