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
package io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.render;

import static java.nio.ByteOrder.LITTLE_ENDIAN;
import static org.lwjgl.opengl.GL11.GL_RGBA;
import static org.lwjgl.opengl.GL11.GL_TEXTURE_2D;
import static org.lwjgl.opengl.GL11.GL_UNSIGNED_BYTE;
import static org.lwjgl.opengl.GL11.glTexImage2D;
import static org.lwjgl.opengl.GL12.GL_BGRA;
import static org.lwjgl.opengl.GL12.GL_TEXTURE_MAX_LEVEL;
import static org.lwjgl.opengl.GL12.GL_TEXTURE_MAX_LOD;
import static org.lwjgl.opengl.GL12.GL_TEXTURE_MIN_LOD;
import static org.lwjgl.opengl.GL14.GL_TEXTURE_LOD_BIAS;

import net.minecraft.client.renderer.GlStateManager;
import net.minecraft.client.renderer.texture.ITextureObject;
import net.minecraft.client.renderer.texture.TextureUtil;
import net.minecraft.client.resources.IResourceManager;
import org.lwjgl.BufferUtils;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;

// TODO: replace with Minecraft's DynamicTexture
public class DynamicTexture implements ITextureObject {

    private final int width, height;
    private int texture;
    private final ByteBuffer data;

    /**
     * @param img BufferedImage to create texture from
     */
    public DynamicTexture(BufferedImage img) {
        // Java:
        // "All BufferedImage objects have an upper left corner coordinate of (0, 0)"
        // OpenGL:
        // "The first element corresponds to the lower left corner of the texture image. Subsequent elements progress left-to-right through
        // the remaining texels in the lowest row of the texture image, and then in successively higher rows of the texture image.
        // The final element corresponds to the upper right corner of the texture image."
        // OpenGL has it upside down, so we have to work around that
        // set little endian order so that it java's ARGB turns into OpenGL's BGRA
        ByteBuffer buffer = BufferUtils.createByteBuffer(img.getWidth() * img.getHeight() * 4);
        IntBuffer intBuf = buffer.order(LITTLE_ENDIAN).asIntBuffer();

        for (int y = img.getHeight() - 1; y >= 0; y--) {
            intBuf.put(img.getRGB(0, y, img.getWidth(), 1, null, 0, img.getWidth()));
        }

        this.data = buffer;
        this.width = img.getWidth();
        this.height = img.getHeight();
    }

    public void delete() {
        if (texture > 0) {
            TextureUtil.deleteTexture(texture);
        }
    }

    @Override public void setBlurMipmap(boolean blurIn, boolean mipmapIn) {
        // no-op
    }

    @Override public void restoreLastBlurMipmap() {
        // no-op
    }

    @Override public void loadTexture(IResourceManager resourceManager) {
        delete();
        int texture = TextureUtil.glGenTextures();
        GlStateManager.bindTexture(texture);
        // TODO: are these 4 needed for anything?
        GlStateManager.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
        GlStateManager.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_LOD, 0);
        GlStateManager.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LOD, 0);
        GlStateManager.glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_LOD_BIAS, 0);
       // GlStateManager.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, glInterpolationMode);
        //GlStateManager.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, glInterpolationMode);

        int internalFmt = GL_RGBA;
        int fmt = GL_BGRA;
        glTexImage2D(GL_TEXTURE_2D, 0, internalFmt, width, height, 0, fmt, GL_UNSIGNED_BYTE, this.data);
        this.texture = texture;
    }

    @Override public int getGlTextureId() {
        return texture;
    }
}
