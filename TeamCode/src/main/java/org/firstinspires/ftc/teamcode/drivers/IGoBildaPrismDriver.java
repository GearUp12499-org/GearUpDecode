package org.firstinspires.ftc.teamcode.drivers;

public interface IGoBildaPrismDriver {
    int getBootAnimationArtboard();

    boolean insertAnimation(GoBildaPrismDriver.LayerHeight height, PrismAnimations.AnimationBase animation);

    boolean insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight height, PrismAnimations.AnimationBase animation);

    boolean updateAllAnimations();

    boolean updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight height);

    void clearAllAnimations();

    void saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard artboard);

    void loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard artboard);

    void setDefaultBootArtboard(GoBildaPrismDriver.Artboard artboard);

    void enableDefaultBootArtboard(boolean enable);
}
