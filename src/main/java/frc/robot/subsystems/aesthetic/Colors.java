package frc.robot.subsystems.aesthetic;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.hal.util.BoundaryException;

import com.ctre.phoenix.led.CANdle.LEDStripType;

import frc.robot.Constants.Lights;

public class Colors {
	public enum Effect {
		LARSON,
		FLOW,
		CHROMA,
		BREATHE,
		REN_SPECIAL
	}

	Lights light = new Lights();
	CANdle rgb = new CANdle(light.id);
	int numLED = light.leds;
	public Colors() {
		CANdleConfiguration config = new CANdleConfiguration();
		config.stripType = LEDStripType.RGB;
		config.brightnessScalar = light.brightnessScalar;

		rgb.configAllSettings(config); //Applies all settings for
	}	

	public void startRGB(Effect effects) {
		// Animation animation = switch (effects) {
		// 	case LARSON -> new LarsonAnimation(255, 255, 255);
		// 	case FLOW -> new ColorFlowAnimation(255, 255, 255);
		// 	case CHROMA -> new RainbowAnimation(0.2,0.2,numLED);
		// 	case BREATHE -> new SingleFadeAnimation(235, 209, 39);
		// 	case REN_SPECIAL -> new TwinkleAnimation(235, 209, 39);
		// };

		// Note: Temporary fix to avoid < Java 14 error  - GO
		Animation animation = new RainbowAnimation(0.2,0.2, numLED);

		rgb.animate(animation);
	}	
}
