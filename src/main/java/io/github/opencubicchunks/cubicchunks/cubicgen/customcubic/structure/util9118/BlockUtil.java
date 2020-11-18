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

import net.minecraft.block.Block;
import net.minecraft.block.state.IBlockState;

public class BlockUtil 
{
	//returns state from string with format of mod:block:meta with meta being optional
	//may return null
	@SuppressWarnings("deprecation")
	public static IBlockState getStateFromString(String block)
	{
		String[] splitty = block.split(":");
		Block blocky;
		if(splitty.length > 2)
		{
			blocky = Block.getBlockFromName(splitty[0] + ":" + splitty[1]);
			return blocky==null? null : blocky.getStateFromMeta(Integer.valueOf(splitty[2]));
		}
		else
		{
			blocky = Block.getBlockFromName(block);
			return blocky==null? null : blocky.getDefaultState();
		}
	}

}