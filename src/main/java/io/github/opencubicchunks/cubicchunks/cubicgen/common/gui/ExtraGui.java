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
package io.github.opencubicchunks.cubicchunks.cubicgen.common.gui;

import io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.component.IDragTickable;
import io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.component.UILayout;
import net.malisis.core.client.gui.MalisisGui;
import net.malisis.core.client.gui.component.UIComponent;
import net.malisis.core.client.gui.component.container.UIContainer;
import net.malisis.core.util.MouseButton;
import org.lwjgl.input.Keyboard;
import org.lwjgl.input.Mouse;

import java.lang.ref.WeakReference;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.WeakHashMap;

public abstract class ExtraGui extends MalisisGui {

    private Map<IDragTickable, DragTickableWrapper> set = new WeakHashMap<>();
    private Set<UIComponent<?>> addedComponents = new HashSet<>();

    private final Field componentsField;

    private boolean debug = false;

    private List<UIComponent<?>> toAddLater = new ArrayList<>();

    {
        try {
            componentsField = UIContainer.class.getDeclaredField("components");
        } catch (NoSuchFieldException e) {
            throw new Error(e);
        }
        componentsField.setAccessible(true);

        this.registerKeyListener((character, keyCode) -> {
            if (Keyboard.isKeyDown(Keyboard.KEY_LMENU) && keyCode == Keyboard.KEY_P) {
                debug = !debug;
                return true;
            }
            if (debug && keyCode == Keyboard.KEY_L) {
                addedComponents.forEach(this::layout);
                return true;
            }
            return false;
        });
    }

    // a workaround to make UISelect have correct Z order
    public <T> void delayedAdd(UIComponent<?> toAdd) {
        this.toAddLater.add(toAdd);
    }

    protected void afterConstruct() {
        this.toAddLater.forEach(this::addToScreen);
        this.toAddLater.clear();
    }

    @Override public void addToScreen(UIComponent<?> component) {
        addedComponents.add(component);
        super.addToScreen(component);
    }

    @Override public void removeFromScreen(UIComponent<?> component) {
        addedComponents.remove(component);
        super.removeFromScreen(component);
    }

    @Override public void close() {
        // do proper cleanup, because there are static variables (and weak hash maps) that keep track of everything
        draggedComponent = null;
        tooltipComponent = null;
        this.clearScreen();

        super.close();
    }

    @Override public void clearScreen() {
        addedComponents.clear();
        set.clear();
        super.clearScreen();
    }

    @Override
    public void update(int mouseX, int mouseY, float partialTick) {
        set.values().removeIf(wrapper -> !wrapper.tick(mouseX, mouseY, partialTick));

        toAddLater.forEach(this::addToScreen);
        toAddLater.clear();

        if (!debug) {
            addedComponents.forEach(this::layout);
        }
    }

    private void layout(UIComponent<?> comp) {
        // layout() parent twice as re-layout for children may change result for the parent
        if (comp instanceof UILayout<?>) {
            ((UILayout<?>) comp).checkLayout();
        }
        if (comp instanceof UIContainer<?>) {
            UIContainer<?> cont = (UIContainer<?>) comp;
            Set<UIComponent<?>> components = null;
            try {
                components = (Set<UIComponent<?>>) componentsField.get(cont);
            } catch (IllegalAccessException e) {
                throw new Error(e);
            }
            components.forEach(this::layout);
        }
        if (comp instanceof UILayout<?>) {
            ((UILayout<?>) comp).checkLayout();
        }
    }

    public <T extends UIComponent<X> & IDragTickable, X extends UIComponent<X>> void registerDragTickable(T t) {
        set.put(t, new DragTickableWrapper(t));
    }

    public static final class DragTickableWrapper {

        private final WeakReference<IDragTickable> component;
        private boolean beforeClickHovered = false;

        public DragTickableWrapper(IDragTickable component) {
            this.component = new WeakReference<>(component);
        }

        boolean tick(int mouseX, int mouseY, float partialTick) {
            IDragTickable tickable = component.get();
            if (tickable == null) {
                return false;
            }
            if (((UIComponent<?>) tickable).isFocused() && beforeClickHovered && Mouse.isButtonDown(MouseButton.LEFT.getCode())) {
                tickable.onDragTick(mouseX, mouseY, partialTick);
            } else {
                beforeClickHovered = ((UIComponent<?>) tickable).isHovered();
            }
            return true;
        }
    }
}
