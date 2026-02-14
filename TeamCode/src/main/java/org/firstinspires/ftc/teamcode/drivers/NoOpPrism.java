package org.firstinspires.ftc.teamcode.drivers;

public class NoOpPrism implements IGoBildaPrismDriver {
    public static final NoOpPrism INSTANCE = new NoOpPrism();

    @Override
    public int getBootAnimationArtboard() {
        return 0;
    }

    @Override
    public boolean insertAnimation(GoBildaPrismDriver.LayerHeight height, PrismAnimations.AnimationBase animation) {
        return false;
    }

    @Override
    public boolean insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight height, PrismAnimations.AnimationBase animation) {
        return false;
    }

    @Override
    public boolean updateAllAnimations() {
        return false;
    }

    @Override
    public boolean updateAnimationFromIndex(GoBildaPrismDriver.LayerHeight height) {
        return false;
    }

    @Override
    public void clearAllAnimations() {

    }

    @Override
    public void saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard artboard) {

    }

    @Override
    public void loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard artboard) {

    }

    @Override
    public void setDefaultBootArtboard(GoBildaPrismDriver.Artboard artboard) {

    }

    @Override
    public void enableDefaultBootArtboard(boolean enable) {

    }
}
