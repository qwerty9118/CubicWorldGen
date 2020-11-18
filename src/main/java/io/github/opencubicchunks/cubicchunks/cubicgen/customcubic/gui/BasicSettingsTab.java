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

import blue.endless.jankson.JsonObject;
import blue.endless.jankson.JsonPrimitive;
import io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.BiomeOption;
import io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.ExtraGui;
import io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.component.UIVerticalTableLayout;
import io.github.opencubicchunks.cubicchunks.cubicgen.preset.JsonObjectView;
import net.malisis.core.client.gui.component.UIComponent;
import net.malisis.core.client.gui.component.interaction.UICheckBox;
import net.malisis.core.client.gui.component.interaction.UISelect;
import net.malisis.core.client.gui.component.interaction.UISlider;
import net.minecraft.world.biome.Biome;

import static io.github.opencubicchunks.cubicchunks.cubicgen.common.gui.MalisisGuiUtils.*;
import static io.github.opencubicchunks.cubicchunks.cubicgen.customcubic.gui.CustomCubicGui.*;

class BasicSettingsTab {

    private final UIVerticalTableLayout container;

    private final UICheckBox caves;
    private final UICheckBox strongholds;
    private final UICheckBox villages;
    private final UICheckBox mineshafts;
    private final UICheckBox temples;
    private final UICheckBox ravines;
    private final UICheckBox oceanMonuments;
    private final UICheckBox woodlandMansions;
    private final UICheckBox dungeons;
    private final UICheckBox lavaOceans;

    private final UISelect<BiomeOption> biome;

    private final UISlider<Integer> dungeonCount;

    private final UISlider<Integer> biomeSize;
    private final UISlider<Integer> riverSize;

    private final UISlider<Float> waterLevel;

    BasicSettingsTab(ExtraGui gui, JsonObjectView conf) {

        UIVerticalTableLayout<?> layout = new UIVerticalTableLayout<>(gui, 6)
                .setPadding(HORIZONTAL_PADDING, 0)
                .setSize(UIComponent.INHERITED, UIComponent.INHERITED)
                .setInsets(VERTICAL_INSETS, VERTICAL_INSETS, HORIZONTAL_INSETS, HORIZONTAL_INSETS)
                .setRightPadding(HORIZONTAL_PADDING + 6)

                .add(this.caves = makeCheckbox(gui, malisisText("caves"), conf.getBool("caves")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 0, 0, WIDTH_2_COL))
                .add(this.strongholds = makeCheckbox(gui, malisisText("strongholds"), conf.getBool("strongholds")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 1, 0, WIDTH_2_COL))

                .add(this.villages = makeCheckbox(gui, malisisText("villages"), conf.getBool("villages")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 0, 1, WIDTH_2_COL))
                .add(this.mineshafts = makeCheckbox(gui, malisisText("mineshafts"), conf.getBool("mineshafts")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 1, 1, WIDTH_2_COL))

                .add(this.temples = makeCheckbox(gui, malisisText("temples"), conf.getBool("temples")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 0, 2, WIDTH_2_COL))
                .add(this.ravines = makeCheckbox(gui, malisisText("ravines"), conf.getBool("ravines")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 1, 2, WIDTH_2_COL))

                .add(this.oceanMonuments = makeCheckbox(gui, malisisText("oceanMonuments"), conf.getBool("oceanMonuments")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 0, 3, WIDTH_2_COL))
                .add(this.woodlandMansions = makeCheckbox(gui, malisisText("woodlandMansions"), conf.getBool("woodlandMansions")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 1, 3, WIDTH_2_COL))


                .add(this.dungeons = makeCheckbox(gui, malisisText("dungeons"), conf.getBool("dungeons")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 0, 4, WIDTH_2_COL))

                .add(this.lavaOceans = makeCheckbox(gui, malisisText("lavaOceans"), conf.getBool("lavaOceans")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 1, 4, WIDTH_2_COL))


                .add(this.biome = makeBiomeList(gui, conf.getInt("biome")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 0, 5, WIDTH_2_COL))
                .add(this.dungeonCount = makeIntSlider(gui, malisisText("dungeonCount", ": %d"), 1, 100, conf.getInt("dungeonCount")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 1, 5, WIDTH_2_COL))
                
                .add(this.biomeSize = makeIntSlider(gui, malisisText("biomeSize", ": %d"), 1, 8, conf.getInt("biomeSize")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 0, 6, WIDTH_2_COL))
                .add(this.riverSize = makeIntSlider(gui, malisisText("riverSize", ": %d"), 1, 5, conf.getInt("riverSize")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 1, 6, WIDTH_2_COL))


                .add(this.waterLevel = makeExponentialSlider(
                        gui, malisisText("water_level", ": %.2f"),
                        1, 12, 1, 12, conf.getInt("waterLevel")),
                        new UIVerticalTableLayout.GridLocation(WIDTH_2_COL * 0, 7, WIDTH_2_COL));


        this.container = layout;
    }

    UIVerticalTableLayout getContainer() {
        return container;
    }

    void writeConfig(JsonObjectView conf) {
        conf.put("caves", caves.isChecked());
        conf.put("strongholds", strongholds.isChecked());
        conf.put("villages", villages.isChecked());
        conf.put("mineshafts", mineshafts.isChecked());
        conf.put("temples", temples.isChecked());
        conf.put("ravines", ravines.isChecked());
        conf.put("oceanMonuments", oceanMonuments.isChecked());
        conf.put("woodlandMansions", woodlandMansions.isChecked());
        conf.put("dungeons", dungeons.isChecked());
        conf.put("lavaOceans", lavaOceans.isChecked());
        conf.put("biome", biome.getSelectedValue().getBiome() == null ? -1 : Biome.getIdForBiome(biome.getSelectedValue().getBiome()));
        conf.put("dungeonCount", dungeonCount.getValue());
        conf.put("biomeSize", biomeSize.getValue());
        conf.put("riverSize", riverSize.getValue());
        conf.put("waterLevel", Math.round(waterLevel.getValue()));

        String liquidName = lavaOceans.isChecked() ? "minecraft:lava" : "minecraft:water";
        JsonObject liquid = new JsonObject();
        JsonObject liquidProp = new JsonObject();
        liquidProp.put("level", new JsonPrimitive(0.0));
        liquid.put("Properties", liquidProp);
        liquid.put("Name", new JsonPrimitive(liquidName));

        conf.object("replacerConfig").object("defaults").put("cubicgen:water_level", waterLevel.getValue());
        conf.object("replacerConfig").object("defaults").put("cubicgen:ocean_block", liquid);
    }

    public double getWaterLevel() {
        return waterLevel.getValue();
    }
}
