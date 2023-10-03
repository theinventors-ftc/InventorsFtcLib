package org.inventors.ftc.robotbase.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RevLedBlinker{

//               **Colour Palette**
//            .RAINBOW_RAINBOW_PALETTE
//            .RAINBOW_PARTY_PALETTE
//            .RAINBOW_OCEAN_PALETTE
//            .RAINBOW_LAVA_PALETTE
//            .RAINBOW_FOREST_PALETTE
//            .RAINBOW_WITH_GLITTER
//            .CONFETTI
//            .SHOT_RED
//            .SHOT_BLUE
//            .SHOT_WHITE
//            .SINELON_RAINBOW_PALETTE
//            .SINELON_PARTY_PALETTE
//            .SINELON_OCEAN_PALETTE
//            .SINELON_LAVA_PALETTE
//            .SINELON_FOREST_PALETTE
//            .BEATS_PER_MINUTE_RAINBOW_PALETTE
//            .BEATS_PER_MINUTE_PARTY_PALETTE
//            .BEATS_PER_MINUTE_OCEAN_PALETTE
//            .BEATS_PER_MINUTE_LAVA_PALETTE
//            .BEATS_PER_MINUTE_FOREST_PALETTE
//            .FIRE_MEDIUM
//            .FIRE_LARGE
//            .TWINKLES_RAINBOW_PALETTE
//            .TWINKLES_PARTY_PALETTE
//            .TWINKLES_OCEAN_PALETTE
//            .TWINKLES_LAVA_PALETTE
//            .TWINKLES_FOREST_PALETTE
//            .COLOR_WAVES_RAINBOW_PALETTE
//            .COLOR_WAVES_PARTY_PALETTE
//            .COLOR_WAVES_OCEAN_PALETTE
//            .COLOR_WAVES_LAVA_PALETTE
//            .COLOR_WAVES_FOREST_PALETTE
//            .LARSON_SCANNER_RED
//            .LARSON_SCANNER_GRAY
//            .LIGHT_CHASE_RED
//            .LIGHT_CHASE_BLUE
//            .LIGHT_CHASE_GRAY
//            .HEARTBEAT_RED
//            .HEARTBEAT_BLUE
//            .HEARTBEAT_WHITE
//            .HEARTBEAT_GRAY
//            .BREATH_RED
//            .BREATH_BLUE
//            .BREATH_GRAY
//            .STROBE_RED
//            .STROBE_BLUE
//            .STROBE_GOLD
//            .STROBE_WHITE
//            .CP1_END_TO_END_BLEND_TO_BLACK
//            .CP1_LARSON_SCANNER
//            .CP1_LIGHT_CHASE
//            .CP1_HEARTBEAT_SLOW
//            .CP1_HEARTBEAT_MEDIUM
//            .CP1_HEARTBEAT_FAST
//            .CP1_BREATH_SLOW
//            .CP1_BREATH_FAST
//            .CP1_SHOT
//            .CP1_STROBE
//            .CP2_END_TO_END_BLEND_TO_BLACK
//            .CP2_LARSON_SCANNER
//            .CP2_LIGHT_CHASE
//            .CP2_HEARTBEAT_SLOW
//            .CP2_HEARTBEAT_MEDIUM
//            .CP2_HEARTBEAT_FAST
//            .CP2_BREATH_SLOW
//            .CP2_BREATH_FAST
//            .CP2_SHOT
//            .CP2_STROBE
//            .CP1_2_SPARKLE_1_ON_2
//            .CP1_2_SPARKLE_2_ON_1
//            .CP1_2_COLOR_GRADIENT
//            .CP1_2_BEATS_PER_MINUTE
//            .CP1_2_END_TO_END_BLEND_1_TO_2
//            .CP1_2_END_TO_END_BLEND
//            .CP1_2_NO_BLENDING
//            .CP1_2_TWINKLES
//            .CP1_2_COLOR_WAVES
//            .CP1_2_SINELON
//            .HOT_PINK
//            .DARK_RED
//            .RED
//            .RED_ORANGE
//            .ORANGE
//            .GOLD
//            .YELLOW
//            .LAWN_GREEN
//            .LIME
//            .DARK_GREEN
//            .GREEN
//            .BLUE_GREEN
//            .AQUA
//            .SKY_BLUE
//            .DARK_BLUE
//            .BLUE
//            .BLUE_VIOLET
//            .VIOLET
//            .WHITE
//            .GRAY
//            .DARK_GRAY
//            .BLACK

    private static final RevBlinkinLedDriver.BlinkinPattern offPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;

    private final RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern currPattern = offPattern;

    public RevLedBlinker(HardwareMap hardwareMap, String instanceName)
    {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, instanceName);
        setPattern(currPattern);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern)
    {
        currPattern = pattern == null ? offPattern : pattern;
        blinkinLedDriver.setPattern(currPattern);
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern()
    {
        return currPattern;
    }
}