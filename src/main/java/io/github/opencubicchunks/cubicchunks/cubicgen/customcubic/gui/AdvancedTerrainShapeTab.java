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

import static io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.MalisisGuiUtils.floatInput;
import static io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.MalisisGuiUtils.label;
import static io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.MalisisGuiUtils.makeCheckbox;
import static io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.MalisisGuiUtils.makeExponentialSlider;
import static io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.MalisisGuiUtils.makeFloatSlider;
import static io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.MalisisGuiUtils.makeIntSlider;
import static io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.MalisisGuiUtils.makeInvertedExponentialSlider;
import static io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.MalisisGuiUtils.makeUISelect;
import static io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.MalisisGuiUtils.malisisText;
import static io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.gui.CustomCubicGui.HORIZONTAL_INSETS;
import static io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.gui.CustomCubicGui.HORIZONTAL_PADDING;
import static io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.gui.CustomCubicGui.VERTICAL_INSETS;
import static io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.gui.CustomCubicGui.WIDTH_1_COL;
import static io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.gui.CustomCubicGui.WIDTH_2_COL;
import static io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.gui.CustomCubicGui.WIDTH_3_COL;

import com.google.common.eventbus.Subscribe;
import io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.component.UISplitLayout;
import io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.component.UIVerticalTableLayout;
import io.github.opencubicchunks.cubicchunks.cubicgen.preset.JsonObjectView;
import net.malisis.core.client.gui.component.UIComponent;
import net.malisis.core.client.gui.component.container.UIContainer;
import net.malisis.core.client.gui.component.interaction.UICheckBox;
import net.malisis.core.client.gui.component.interaction.UISelect;
import net.malisis.core.client.gui.component.interaction.UISlider;
import net.malisis.core.client.gui.component.interaction.UITextField;
import net.malisis.core.client.gui.event.ComponentEvent;
import net.minecraft.util.EnumFacing;
import net.minecraftforge.fml.common.registry.ForgeRegistries;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

class AdvancedTerrainShapeTab {

    private final UIContainer<?> container;

    private final UICheckBox lockExpectedHeights;
    private final UITextField expectedBaseHeight;
    private final UITextField expectedHeightVariation;
    private final UITextField actualHeight;

    private final UISlider<Float> heightVariationFactor;
    private final UISlider<Float> heightVariationSpecialFactor;
    private final UISlider<Float> heightVariationOffset;
    private final UISlider<Float> heightFactor;
    private final UISlider<Float> heightOffset;

    private final UISlider<Float> depthNoisePeriodX;
    private final UISlider<Float> depthNoisePeriodZ;
    private final UISlider<Integer> depthNoiseOctaves;
    private final UISlider<Float> depthNoiseFactor;
    private final UISlider<Float> depthNoiseOffset;

    private final UISlider<Float> selectorNoisePeriodX;
    private final UISlider<Float> selectorNoisePeriodY;
    private final UISlider<Float> selectorNoisePeriodZ;
    private final UISlider<Integer> selectorNoiseOctaves;
    private final UISlider<Float> selectorNoiseFactor;
    private final UISlider<Float> selectorNoiseOffset;

    private final UISlider<Float> lowNoisePeriodX;
    private final UISlider<Float> lowNoisePeriodY;
    private final UISlider<Float> lowNoisePeriodZ;
    private final UISlider<Integer> lowNoiseOctaves;
    private final UISlider<Float> lowNoiseFactor;
    private final UISlider<Float> lowNoiseOffset;

    private final UISlider<Float> highNoisePeriodX;
    private final UISlider<Float> highNoisePeriodY;
    private final UISlider<Float> highNoisePeriodZ;
    private final UISlider<Integer> highNoiseOctaves;
    private final UISlider<Float> highNoiseFactor;
    private final UISlider<Float> highNoiseOffset;

    // preview
    private final UICheckBox keepPreviewVisible;
    private final UISlider<Float> biomeScaleSlider, biomeOffsetSlider;
    private final UISelect<EnumFacing.Axis> horizontalAxis;
    private final UICheckBox lockXZ;
    private final UICheckBox showPreview;
    private final DoubleSupplier getWaterLevel;

    AdvancedTerrainShapeTab(CustomCubicGui gui, JsonObjectView conf, DoubleSupplier getWaterLevel) {
        this.getWaterLevel = getWaterLevel;
        final float MAX_NOISE_FREQ_POWER = -4;

        int gridY = -1;

        String PERIOD_FMT = " %.2f";
        UIVerticalTableLayout<?> table = new UIVerticalTableLayout(gui, 6);
        table.setPadding(HORIZONTAL_PADDING, 0);

        table.setInsets(VERTICAL_INSETS, VERTICAL_INSETS, HORIZONTAL_INSETS, HORIZONTAL_INSETS)
                .setRightPadding(6)

                //expected heights
                .add(label(gui, malisisText("expected_heights_group")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_1_COL * 0, gridY += 2, WIDTH_1_COL))
                .add(this.lockExpectedHeights = makeCheckbox(gui, malisisText("lock_expected_heights"), true),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 0, ++gridY, WIDTH_2_COL))
                .add(floatInput(gui, malisisText("actual_height"),
                        this.actualHeight = new UITextField(gui, ""), conf.getFloat("actualHeight")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 1, gridY, WIDTH_2_COL))
                .add(floatInput(gui, malisisText("expected_base_height"),
                        this.expectedBaseHeight = new UITextField(gui, ""), conf.getFloat("expectedBaseHeight")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 0, ++gridY, WIDTH_2_COL))
                .add(floatInput(gui, malisisText("expected_height_variation"),
                        this.expectedHeightVariation = new UITextField(gui, ""), conf.getFloat("expectedHeightVariation")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 1, gridY, WIDTH_2_COL))
                // height variation
                .add(label(gui, malisisText("height_variation_group")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_1_COL * 0, ++gridY, WIDTH_1_COL))
                .add(this.heightVariationFactor = makeExponentialSlider(
                        gui, malisisText("height_variation_factor_slider", ": %.2f"),
                        Float.NaN, Float.NaN, 0, 20, conf.getFloat("heightVariationFactor")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 0, ++gridY, WIDTH_3_COL))
                .add(this.heightVariationSpecialFactor = makeExponentialSlider(
                        gui, malisisText("height_variation_special_factor_slider", ": %.2f"),
                        Float.NaN, Float.NaN, -6, 6, conf.getFloat("specialHeightVariationFactorBelowAverageY")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 1, gridY, WIDTH_3_COL))
                .add(this.heightVariationOffset = makeExponentialSlider(
                        gui, malisisText("height_variation_offset_slider", ": %.2f"),
                        0, 20, 0, 20, conf.getFloat("heightVariationOffset")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 2, gridY, WIDTH_3_COL))

                // height
                .add(label(gui, malisisText("height_group")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_1_COL * 0, ++gridY, WIDTH_1_COL))
                .add(this.heightFactor = makeExponentialSlider(
                        gui, malisisText("height_factor", ": %.2f"),
                        1, 20, 1, 20, conf.getFloat("heightFactor")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 0, ++gridY, WIDTH_2_COL))
                .add(this.heightOffset = makeExponentialSlider(
                        gui, malisisText("height_offset", ": %.2f"),
                        1, 20, 1, 20, conf.getFloat("heightOffset")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 1, gridY, WIDTH_2_COL))

                // depth noise
                .add(label(gui, malisisText("depth_noise_group")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_1_COL * 0, ++gridY, WIDTH_1_COL))
                .add(this.depthNoisePeriodX = makeInvertedExponentialSlider(
                        gui, malisisText("depth_noise_period_x", PERIOD_FMT),
                        Float.NaN, Float.NaN, -8, MAX_NOISE_FREQ_POWER, 1.0f / conf.getFloat("depthNoiseFrequencyX")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 0, ++gridY, WIDTH_2_COL))
                .add(this.depthNoisePeriodZ = makeInvertedExponentialSlider(
                        gui, malisisText("depth_noise_period_z", PERIOD_FMT),
                        Float.NaN, Float.NaN, -8, MAX_NOISE_FREQ_POWER, 1.0f / conf.getFloat("depthNoiseFrequencyZ")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 1, gridY, WIDTH_2_COL))

                .add(this.depthNoiseOctaves = makeIntSlider(
                        gui, malisisText("depth_noise_octaves", ": %d"),
                        1, 16, conf.getInt("depthNoiseOctaves")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 0, ++gridY, WIDTH_3_COL))
                .add(this.depthNoiseFactor = makeExponentialSlider(
                        gui, malisisText("depth_noise_factor", ": %.4f"),
                        Float.NaN, Float.NaN, 1, 12, conf.getFloat("depthNoiseFactor")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 1, gridY, WIDTH_3_COL))
                .add(this.depthNoiseOffset = makeExponentialSlider(
                        gui, malisisText("depth_noise_offset", ": %.2f"),
                        1, 12, 1, 12, conf.getFloat("depthNoiseOffset")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 2, gridY, WIDTH_3_COL))

                // selector noise
                .add(label(gui, malisisText("selector_noise_group")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_1_COL * 0, ++gridY, WIDTH_1_COL))
                .add(this.selectorNoisePeriodX = makeInvertedExponentialSlider(
                        gui, malisisText("selector_noise_period_x", PERIOD_FMT),
                        Float.NaN, Float.NaN, -8, MAX_NOISE_FREQ_POWER, 1.0f / conf.getFloat("selectorNoiseFrequencyX")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 0, ++gridY, WIDTH_3_COL))
                .add(this.selectorNoisePeriodY = makeInvertedExponentialSlider(
                        gui, malisisText("selector_noise_period_y", PERIOD_FMT),
                        Float.NaN, Float.NaN, -8, MAX_NOISE_FREQ_POWER, 1.0f / conf.getFloat("selectorNoiseFrequencyY")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 1, gridY, WIDTH_3_COL))
                .add(this.selectorNoisePeriodZ = makeInvertedExponentialSlider(
                        gui, malisisText("selector_noise_period_z", PERIOD_FMT),
                        Float.NaN, Float.NaN, -8, MAX_NOISE_FREQ_POWER, 1.0f / conf.getFloat("selectorNoiseFrequencyZ")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 2, gridY, WIDTH_3_COL))

                .add(this.selectorNoiseOctaves = makeIntSlider(
                        gui, malisisText("selector_noise_octaves", ": %d"),
                        1, 16, conf.getInt("selectorNoiseOctaves")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 0, ++gridY, WIDTH_3_COL))
                .add(this.selectorNoiseFactor = makeExponentialSlider(
                        gui, malisisText("selector_noise_factor", ": %.4f"),
                        Float.NaN, Float.NaN, 0, 10, conf.getFloat("selectorNoiseFactor")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 1, gridY, WIDTH_3_COL))
                .add(this.selectorNoiseOffset = makeExponentialSlider(
                        gui, malisisText("selector_noise_offset", ": %.2f"),
                        -5, 5, -5, 5, conf.getFloat("selectorNoiseOffset")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 2, gridY, WIDTH_3_COL))


                // low noise
                .add(label(gui, malisisText("low_noise_group")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_1_COL * 0, ++gridY, WIDTH_1_COL))
                .add(this.lowNoisePeriodX = makeInvertedExponentialSlider(
                        gui, malisisText("low_noise_period_x", PERIOD_FMT),
                        Float.NaN, Float.NaN, -8, MAX_NOISE_FREQ_POWER, 1.0f / conf.getFloat("lowNoiseFrequencyX")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 0, ++gridY, WIDTH_3_COL))
                .add(this.lowNoisePeriodY = makeInvertedExponentialSlider(
                        gui, malisisText("low_noise_period_y", PERIOD_FMT),
                        Float.NaN, Float.NaN, -8, MAX_NOISE_FREQ_POWER, 1.0f / conf.getFloat("lowNoiseFrequencyY")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 1, gridY, WIDTH_3_COL))
                .add(this.lowNoisePeriodZ = makeInvertedExponentialSlider(
                        gui, malisisText("low_noise_period_z", PERIOD_FMT),
                        Float.NaN, Float.NaN, -8, MAX_NOISE_FREQ_POWER, 1.0f / conf.getFloat("lowNoiseFrequencyZ")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 2, gridY, WIDTH_3_COL))

                .add(this.lowNoiseOctaves = makeIntSlider(
                        gui, malisisText("low_noise_octaves", ": %d"),
                        1, 16, conf.getInt("lowNoiseOctaves")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 0, ++gridY, WIDTH_3_COL))
                .add(this.lowNoiseFactor = makeExponentialSlider(
                        gui, malisisText("low_noise_factor", ": %.4f"),
                        Float.NaN, Float.NaN, -10, 10, conf.getFloat("lowNoiseFactor")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 1, gridY, WIDTH_3_COL))
                .add(this.lowNoiseOffset = makeExponentialSlider(
                        gui, malisisText("low_noise_offset", ": %.2f"),
                        -5, 5, -5, 5, conf.getFloat("lowNoiseOffset")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 2, gridY, WIDTH_3_COL))

                // high noise
                .add(label(gui, malisisText("high_noise_group")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_1_COL * 0, ++gridY, WIDTH_1_COL))
                .add(this.highNoisePeriodX = makeInvertedExponentialSlider(
                        gui, malisisText("high_noise_period_x", PERIOD_FMT),
                        Float.NaN, Float.NaN, -8, MAX_NOISE_FREQ_POWER, 1.0f / conf.getFloat("highNoiseFrequencyX")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 0, ++gridY, WIDTH_3_COL))
                .add(this.highNoisePeriodY = makeInvertedExponentialSlider(
                        gui, malisisText("high_noise_period_y", PERIOD_FMT),
                        Float.NaN, Float.NaN, -8, MAX_NOISE_FREQ_POWER, 1.0f / conf.getFloat("highNoiseFrequencyY")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 1, gridY, WIDTH_3_COL))
                .add(this.highNoisePeriodZ = makeInvertedExponentialSlider(
                        gui, malisisText("high_noise_period_z", PERIOD_FMT),
                        Float.NaN, Float.NaN, -8, MAX_NOISE_FREQ_POWER, 1.0f / conf.getFloat("highNoiseFrequencyZ")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 2, gridY, WIDTH_3_COL))

                .add(this.highNoiseOctaves = makeIntSlider(
                        gui, malisisText("high_noise_octaves", ": %d"),
                        1, 16, conf.getInt("highNoiseOctaves")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 0, ++gridY, WIDTH_3_COL))
                .add(this.highNoiseFactor = makeExponentialSlider(
                        gui, malisisText("high_noise_factor", ": %.4f"),
                        Float.NaN, Float.NaN, -10, 10, conf.getFloat("highNoiseFactor")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 1, gridY, WIDTH_3_COL))
                .add(this.highNoiseOffset = makeExponentialSlider(
                        gui, malisisText("high_noise_offset", ": %.2f"),
                        -5, 5, -5, 5, conf.getFloat("highNoiseOffset")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_3_COL * 2, gridY, WIDTH_3_COL));

        final int previewHeight = 100;
        final int settingsSize = 150;
        UITerrainPreview preview;

        gridY = -1;
        final float biomeCount = ForgeRegistries.BIOMES.getValues().size();

        UIContainer<?> settingsContrainer = new UIVerticalTableLayout<>(gui, 4)
                .setInsets(1, 1, 0, 0)
                .add(keepPreviewVisible = makeCheckbox(gui, malisisText("keep_preview_visible"), true),
                        new UIVerticalTableLayout.GridLocation(0, ++gridY, 4))
                .add(showPreview = makeCheckbox(gui, malisisText("show_preview"), true),
                        new UIVerticalTableLayout.GridLocation(0, ++gridY, 4))
                .add(biomeScaleSlider = makeInvertedExponentialSlider(gui, malisisText("biome_scale", ": %.2f"),
                        Float.NaN, Float.NaN, -10, -6, 64), new UIVerticalTableLayout.GridLocation(0, ++gridY, 4))
                .add(biomeOffsetSlider = makeFloatSlider(gui, malisisText("biome_offset", ": %.2f"),
                        0, biomeCount, 0), new UIVerticalTableLayout.GridLocation(0, ++gridY, 4))
                .add(lockXZ = makeCheckbox(gui, malisisText("lock_xz_together"), true),
                        new UIVerticalTableLayout.GridLocation(0, ++gridY, 4))
                .add(horizontalAxis = makeUISelect(gui, Arrays.asList(EnumFacing.Axis.X, EnumFacing.Axis.Z))
                                .setLabelPattern(malisisText("preview_horizontal_axis", ": %s")),
                        new UIVerticalTableLayout.GridLocation(0, ++gridY, 4));
        settingsContrainer.setSize(settingsSize, previewHeight);


        UISplitLayout<?> previewSplitView = new UISplitLayout<>(gui, UISplitLayout.Type.SIDE_BY_SIDE, null, null)
                .setSeparatorSize(4)
                .setMinimumUserComponentSize(UISplitLayout.Pos.FIRST, 50)
                .setMinimumUserComponentSize(UISplitLayout.Pos.SECOND, 150)
                .userResizable(true)
                .setSizeOf(UISplitLayout.Pos.SECOND, 150);

        UITerrainPreview.TerrainPreviewDataAccess dataAccess = makeDataAccess();
        previewSplitView.add(preview = new UITerrainPreview(gui, dataAccess).setSize(UIComponent.INHERITED - settingsSize, UIComponent.INHERITED),
                UISplitLayout.Pos.FIRST);
        previewSplitView.add(settingsContrainer, UISplitLayout.Pos.SECOND);

        UISplitLayout<?> rootSplit = new UISplitLayout<>(gui, UISplitLayout.Type.STACKED, previewSplitView, table);
        rootSplit.setSize(UIComponent.INHERITED, UIComponent.INHERITED).setMinimumUserComponentSize(UISplitLayout.Pos.SECOND, 64);

        heightFactor.register(new Object() {
            @Subscribe
            public void onUpdate(ComponentEvent.ValueChange<UISlider<Float>, Float> evt) {
                updateExpectedHeightsIfNeeded();
            }
        });
        heightOffset.register(new Object() {
            @Subscribe
            public void onUpdate(ComponentEvent.ValueChange<UISlider<Float>, Float> evt) {
                updateExpectedHeightsIfNeeded();
            }
        });
        lockExpectedHeights.register(new Object() {
            @Subscribe
            public void onUpdate(UICheckBox.CheckEvent evt) {
                updateExpectedHeightsIfNeeded();
            }
        });
        biomeScaleSlider.register(new Object() {
            @Subscribe
            public void onUpdate(ComponentEvent.ValueChange<UISlider<Float>, Float> evt) {
                preview.setBiomeScale(evt.getNewValue());
            }
        });
        biomeOffsetSlider.register(new Object() {
            @Subscribe
            public void onUpdate(ComponentEvent.ValueChange<UISlider<Float>, Float> evt) {
                preview.setBiomeOffset(evt.getNewValue());
            }
        });
        horizontalAxis.register(new Object() {
            @Subscribe
            public void onUpdate(ComponentEvent.ValueChange<UISelect<EnumFacing.Axis>, EnumFacing.Axis> evt) {
                preview.setShownAxis(evt.getNewValue());
            }
        });
        lockXZ.register(new Object() {
            @Subscribe
            public void onCheck(UICheckBox.CheckEvent evt) {
                setLockedXZ(evt.isChecked());
            }
        });
        showPreview.register(new Object() {
            @Subscribe
            public void onCheck(UICheckBox.CheckEvent evt) {
                preview.setEnabled(evt.isChecked());
            }
        });
        depthNoisePeriodX.register(new Object() {
            @Subscribe
            public void onUpdate(ComponentEvent.ValueChange<UISlider<Float>, Float> evt) {
                if (lockXZ.isChecked()) {
                    depthNoisePeriodZ.setValue(depthNoisePeriodX.getValue());
                }
            }
        });
        lowNoisePeriodX.register(new Object() {
            @Subscribe
            public void onUpdate(ComponentEvent.ValueChange<UISlider<Float>, Float> evt) {
                if (lockXZ.isChecked()) {
                    lowNoisePeriodZ.setValue(lowNoisePeriodX.getValue());
                }
            }
        });
        highNoisePeriodX.register(new Object() {
            @Subscribe
            public void onUpdate(ComponentEvent.ValueChange<UISlider<Float>, Float> evt) {
                if (lockXZ.isChecked()) {
                    highNoisePeriodZ.setValue(highNoisePeriodX.getValue());
                }
            }
        });
        selectorNoisePeriodX.register(new Object() {
            @Subscribe
            public void onUpdate(ComponentEvent.ValueChange<UISlider<Float>, Float> evt) {
                if (lockXZ.isChecked()) {
                    selectorNoisePeriodZ.setValue(selectorNoisePeriodX.getValue());
                }
            }
        });
        setLockedXZ(lockXZ.isChecked());
        preview.setBiomeScale(biomeScaleSlider.getValue());
        preview.setBiomeOffset(biomeOffsetSlider.getValue());
        horizontalAxis.setSelectedOption(EnumFacing.Axis.X);
        preview.setShownAxis(horizontalAxis.getSelectedValue());
        keepPreviewVisible.register(new Object() {
            @Subscribe
            public void onCheck(UICheckBox.CheckEvent evt) {
                if (evt.isChecked()) {
                    table.setSize(UIComponent.INHERITED, UIComponent.INHERITED - previewSplitView.getHeight());
                    // check if the container is added, because the first time the event is fired it's not added
                    if (previewSplitView.getParent() == table) {
                        table.remove(previewSplitView);
                    }
                    rootSplit.setMinimumUserComponentSize(UISplitLayout.Pos.FIRST, 64)
                            .setSeparatorSize(4)
                            .setSizeOf(UISplitLayout.Pos.FIRST, 64)
                            .userResizable(true);
                    previewSplitView.setPadding(HORIZONTAL_PADDING + HORIZONTAL_INSETS, 2);
                    rootSplit.add(previewSplitView, UISplitLayout.Pos.FIRST);
                } else {
                    if (previewSplitView.getParent() == rootSplit) {
                        rootSplit.remove(previewSplitView);
                    }
                    previewSplitView.setPadding(0, 0);
                    rootSplit.setMinimumUserComponentSize(UISplitLayout.Pos.FIRST, 0)
                            .setSeparatorSize(0)
                            .setSizeOf(UISplitLayout.Pos.FIRST, 0)
                            .userResizable(false);
                    table.setSize(UIComponent.INHERITED, UIComponent.INHERITED);
                    table.add(previewSplitView, new UIVerticalTableLayout.GridLocation(WIDTH_1_COL * 0, 0, WIDTH_1_COL));
                }
            }
        });
        // fire the event to correctly set the layout
        keepPreviewVisible.fireEvent(new UICheckBox.CheckEvent(keepPreviewVisible, keepPreviewVisible.isChecked()));

        this.container = rootSplit;
    }

    private UITerrainPreview.TerrainPreviewDataAccess makeDataAccess() {
        return new UITerrainPreview.TerrainPreviewDataAccess()
                .setWaterLevel(this.getWaterLevel)
                .setHeightVariationFactor(this.heightVariationFactor::getValue)
                .setSpecialHeightVariationFactorBelowAverageY(this.heightVariationSpecialFactor::getValue)
                .setHeightVariationOffset(this.heightVariationOffset::getValue)
                .setHeightFactor(this.heightFactor::getValue)
                .setHeightOffset(this.heightOffset::getValue)
                .setDepthNoiseFactor(this.depthNoiseFactor::getValue)
                .setDepthNoiseOffset(this.depthNoiseOffset::getValue)
                .setDepthNoiseFrequencyX(() -> 1.0 / this.depthNoisePeriodX.getValue())
                .setDepthNoiseFrequencyZ(() -> 1.0 / this.depthNoisePeriodZ.getValue())
                .setDepthNoiseOctaves(this.depthNoiseOctaves::getValue)
                .setSelectorNoiseFactor(this.selectorNoiseFactor::getValue)
                .setSelectorNoiseOffset(this.selectorNoiseOffset::getValue)
                .setSelectorNoiseFrequencyX(() -> 1.0 / this.selectorNoisePeriodX.getValue())
                .setSelectorNoiseFrequencyZ(() -> 1.0 / this.selectorNoisePeriodZ.getValue())
                .setSelectorNoiseFrequencyY(() -> 1.0 / this.selectorNoisePeriodY.getValue())
                .setSelectorNoiseOctaves(this.selectorNoiseOctaves::getValue)
                .setLowNoiseFactor(this.lowNoiseFactor::getValue)
                .setLowNoiseOffset(this.lowNoiseOffset::getValue)
                .setLowNoiseFrequencyX(() -> 1.0 / this.lowNoisePeriodX.getValue())
                .setLowNoiseFrequencyZ(() -> 1.0 / this.lowNoisePeriodZ.getValue())
                .setLowNoiseFrequencyY(() -> 1.0 / this.lowNoisePeriodY.getValue())
                .setLowNoiseOctaves(this.lowNoiseOctaves::getValue)
                .setHighNoiseFactor(this.highNoiseFactor::getValue)
                .setHighNoiseOffset(this.highNoiseOffset::getValue)
                .setHighNoiseFrequencyX(() -> 1.0 / this.highNoisePeriodX.getValue())
                .setHighNoiseFrequencyZ(() -> 1.0 / this.highNoisePeriodZ.getValue())
                .setHighNoiseFrequencyY(() -> 1.0 / this.highNoisePeriodY.getValue())
                .setHighNoiseOctaves(this.highNoiseOctaves::getValue);
    }

    private void updateExpectedHeightsIfNeeded() {
        if (this.lockExpectedHeights.isChecked()) {
            this.expectedBaseHeight.setText(String.format("%.1f", this.heightOffset.getValue()));
            this.expectedHeightVariation.setText(String.format("%.1f", this.heightFactor.getValue()));
            float actualHeight = (this.heightOffset.getValue() + this.heightVariationOffset.getValue() +
                    Math.max(this.heightFactor.getValue() * 2 + this.heightVariationFactor.getValue(),
                            this.heightFactor.getValue() + this.heightVariationFactor.getValue() * 2));
            this.actualHeight.setText(String.format("%.1f", actualHeight));
        }
    }

    private void setLockedXZ(boolean lock) {
        this.depthNoisePeriodZ.setEnabled(!lock);
        this.lowNoisePeriodZ.setEnabled(!lock);
        this.highNoisePeriodZ.setEnabled(!lock);
        this.selectorNoisePeriodZ.setEnabled(!lock);

        if (lock) {
            this.depthNoisePeriodZ.setValue(this.depthNoisePeriodX.getValue());
            this.lowNoisePeriodZ.setValue(this.lowNoisePeriodZ.getValue());
            this.highNoisePeriodZ.setValue(this.highNoisePeriodZ.getValue());
            this.selectorNoisePeriodZ.setValue(this.selectorNoisePeriodZ.getValue());
        }
    }

    UIContainer<?> getContainer() {
        return container;
    }

    DoubleSupplier getExpectedBaseHeight() {
        return () -> tryParseFloatOrDefault(expectedBaseHeight.getText(), Float.NaN);
    }

    DoubleSupplier getExpectedHeightVariation() {
        return () -> tryParseFloatOrDefault(expectedHeightVariation.getText(), Float.NaN);
    }

    private static float tryParseFloatOrDefault(String val, float def) {
        try {
            return Float.parseFloat(val);
        } catch (NumberFormatException ignored) {
        }
        try {
            return (float) Double.parseDouble(val);
        } catch (NumberFormatException ignored) {
        }
        return def;
    }

    void writeConfig(JsonObjectView conf) {
        /* 
         * Possible NumberFormatException errors in TextFields. 
         * - If they happen, turn the Cursor to red, as soon as a value is valid, turn back to default grey.
         * - If the customization is left (by pressing Done) when such an error exists, the value resets to default.
         */
        float expectedBaseHeight, expectedHeightVariation, actualHeight;
        try {
            expectedBaseHeight = Float.parseFloat(this.expectedBaseHeight.getText());
            this.expectedBaseHeight.setCursorColor(0xD0D0D0);
        } catch (NumberFormatException e) {
            this.expectedBaseHeight.setCursorColor(0xD00000);
            return;
        }
        
        try {
            expectedHeightVariation = Float.parseFloat(this.expectedHeightVariation.getText());
            this.expectedHeightVariation.setCursorColor(0xD0D0D0);
        } catch (NumberFormatException e) {
            this.expectedHeightVariation.setCursorColor(0xD00000);
            return;
        }
        
        try {
            actualHeight = Float.parseFloat(this.actualHeight.getText());
            this.actualHeight.setCursorColor(0xD0D0D0);
        } catch (NumberFormatException e) {
            this.actualHeight.setCursorColor(0xD00000);
            return;
        }
        conf.put("expectedBaseHeight", expectedBaseHeight);
        conf.put("expectedHeightVariation", expectedHeightVariation);
        conf.put("actualHeight", actualHeight);

        conf.put("heightVariationFactor", this.heightVariationFactor.getValue());
        conf.put("specialHeightVariationFactorBelowAverageY", this.heightVariationSpecialFactor.getValue());
        conf.put("heightVariationOffset", this.heightVariationOffset.getValue());
        conf.put("heightFactor", this.heightFactor.getValue());
        conf.put("heightOffset", this.heightOffset.getValue());

        conf.put("depthNoiseFrequencyX", 1.0f / this.depthNoisePeriodX.getValue());
        conf.put("depthNoiseFrequencyZ", 1.0f / this.depthNoisePeriodZ.getValue());
        conf.put("depthNoiseOctaves", this.depthNoiseOctaves.getValue());
        conf.put("depthNoiseFactor", this.depthNoiseFactor.getValue());
        conf.put("depthNoiseOffset", this.depthNoiseOffset.getValue());

        conf.put("selectorNoiseFrequencyX", 1.0f / this.selectorNoisePeriodX.getValue());
        conf.put("selectorNoiseFrequencyY", 1.0f / this.selectorNoisePeriodY.getValue());
        conf.put("selectorNoiseFrequencyZ", 1.0f / this.selectorNoisePeriodZ.getValue());
        conf.put("selectorNoiseOctaves", this.selectorNoiseOctaves.getValue());
        conf.put("selectorNoiseFactor", this.selectorNoiseFactor.getValue());
        conf.put("selectorNoiseOffset", this.selectorNoiseOffset.getValue());

        conf.put("lowNoiseFrequencyX", 1.0f / this.lowNoisePeriodX.getValue());
        conf.put("lowNoiseFrequencyY", 1.0f / this.lowNoisePeriodY.getValue());
        conf.put("lowNoiseFrequencyZ", 1.0f / this.lowNoisePeriodZ.getValue());
        conf.put("lowNoiseOctaves", this.lowNoiseOctaves.getValue());
        conf.put("lowNoiseFactor", this.lowNoiseFactor.getValue());
        conf.put("lowNoiseOffset", this.lowNoiseOffset.getValue());

        conf.put("highNoiseFrequencyX", 1.0f / this.highNoisePeriodX.getValue());
        conf.put("highNoiseFrequencyY", 1.0f / this.highNoisePeriodY.getValue());
        conf.put("highNoiseFrequencyZ", 1.0f / this.highNoisePeriodZ.getValue());
        conf.put("highNoiseOctaves", this.highNoiseOctaves.getValue());
        conf.put("highNoiseFactor", this.highNoiseFactor.getValue());
        conf.put("highNoiseOffset", this.highNoiseOffset.getValue());
    }
}
