"""
Advanced Light Control Module for ROV with PWM Support
Controls LED lights with multiple PWM frequencies for different lighting effects.
Supports brightness control, dimming, and various lighting patterns.
"""

import time
import sys
import os
import threading
from typing import Dict, Optional, Union

# Smart import strategy for logging
try:
    # Try absolute import from project root
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
    from src import log
    logger = log.getLogger(__name__)
except ImportError:
    try:
        # Try relative import from src directory
        from .. import log
        logger = log.getLogger(__name__)
    except ImportError:
        # Fallback to standard logging
        import logging
        logging.basicConfig(level=logging.INFO)
        logger = logging.getLogger(__name__)

# Try to import gpiozero first (preferred), then fall back to RPi.GPIO
GPIO_METHOD = None
LED_CONTROLLERS = {}

try:
    from gpiozero import PWMLED, LED
    GPIO_METHOD = "gpiozero"
    logger.info("gpiozero imported successfully - Advanced PWM lighting available")
except ImportError:
    try:
        import RPi.GPIO as GPIO
        GPIO_METHOD = "RPi.GPIO"
        logger.info("RPi.GPIO imported successfully - Basic GPIO lighting available")
    except ImportError:
        GPIO_METHOD = "simulation"
        logger.warning("No GPIO library available - Running in simulation mode")

# PWM Frequency configurations for different lighting effects
PWM_FREQUENCIES = {
    'low': 100,      # 100 Hz - Smooth dimming, good for general lighting
    'medium': 1000,  # 1 kHz - Standard PWM, good balance of smoothness and efficiency
    'high': 10000    # 10 kHz - High frequency, minimal flicker, best for video recording
}

# Light Configuration with PWM support - Single main light with switchable frequencies
LIGHT_CONFIG = {
    'main_light': {
        'pin': 16,
        'pwm_freq': 'medium',  # Default frequency
        'description': 'Main ROV illumination light',
        'max_brightness': 1.0
    }
}

# Ensure LED_CONTROLLERS is initialized
LED_CONTROLLERS = {}

# Global state
lights_initialized = False
light_states = {}
brightness_levels = {}

class LightController:
    """Advanced light controller with PWM support"""
    
    def __init__(self, name: str, config: Dict):
        self.name = name
        self.pin = config['pin']
        self.pwm_freq = config['pwm_freq']
        self.description = config['description']
        self.max_brightness = config['max_brightness']
        self.controller = None
        self.current_brightness = 0.0
        self.is_blinking = False
        self.blink_thread = None
        
    def initialize(self) -> bool:
        """Initialize the light controller"""
        global GPIO_METHOD
        try:
            if GPIO_METHOD == "gpiozero":
                # Use PWMLED for advanced control
                freq_hz = PWM_FREQUENCIES[self.pwm_freq]
                try:
                    self.controller = PWMLED(self.pin, frequency=freq_hz)
                    logger.debug(f"Initialized {self.name} with PWM frequency {freq_hz}Hz (gpiozero)")
                    return True
                except Exception as e:
                    logger.warning(f"gpiozero PWMLED failed ({e}), switching to simulation mode")
                    GPIO_METHOD = "simulation"
                    return True
            elif GPIO_METHOD == "RPi.GPIO":
                # Use basic GPIO with software PWM
                try:
                    GPIO.setup(self.pin, GPIO.OUT)
                    freq_hz = PWM_FREQUENCIES[self.pwm_freq]
                    self.controller = GPIO.PWM(self.pin, freq_hz)
                    self.controller.start(0)
                    logger.debug(f"Initialized {self.name} with RPi.GPIO PWM {freq_hz}Hz")
                    return True
                except Exception as e:
                    logger.warning(f"RPi.GPIO failed ({e}), switching to simulation mode")
                    GPIO_METHOD = "simulation"
                    return True
            else:
                logger.debug(f"Initialized {self.name} in simulation mode")
                return True
        except Exception as e:
            logger.warning(f"Failed to initialize {self.name}: {e}, using simulation mode")
            GPIO_METHOD = "simulation"
            return True
    
    def set_brightness(self, brightness: float) -> bool:
        """Set light brightness (0.0 to 1.0)"""
        try:
            # Clamp brightness to valid range and max brightness
            brightness = max(0.0, min(brightness, self.max_brightness))
            
            if GPIO_METHOD == "gpiozero" and self.controller:
                self.controller.value = brightness
                logger.debug(f"{self.name} brightness set to {brightness:.2f} (gpiozero)")
            elif GPIO_METHOD == "RPi.GPIO" and self.controller:
                duty_cycle = brightness * 100
                self.controller.ChangeDutyCycle(duty_cycle)
                logger.debug(f"{self.name} brightness set to {brightness:.2f} (RPi.GPIO)")
            else:
                logger.debug(f"{self.name} brightness set to {brightness:.2f} (simulated)")
            
            self.current_brightness = brightness
            return True
        except Exception as e:
            logger.error(f"Failed to set {self.name} brightness: {e}")
            return False
    
    def on(self, brightness: Optional[float] = None) -> bool:
        """Turn light on at specified brightness"""
        if brightness is None:
            brightness = self.max_brightness
        return self.set_brightness(brightness)
    
    def off(self) -> bool:
        """Turn light off"""
        self.stop_blink()
        return self.set_brightness(0.0)
    
    def dim(self, target_brightness: float, duration: float = 2.0) -> bool:
        """Gradually dim light to target brightness"""
        try:
            start_brightness = self.current_brightness
            steps = int(duration * 50)  # 50 steps per second
            step_size = (target_brightness - start_brightness) / steps
            step_duration = duration / steps
            
            def dim_thread():
                for i in range(steps + 1):
                    brightness = start_brightness + (step_size * i)
                    self.set_brightness(brightness)
                    time.sleep(step_duration)
            
            thread = threading.Thread(target=dim_thread)
            thread.daemon = True
            thread.start()
            
            logger.info(f"Dimming {self.name} from {start_brightness:.2f} to {target_brightness:.2f} over {duration}s")
            return True
        except Exception as e:
            logger.error(f"Failed to dim {self.name}: {e}")
            return False
    
    def blink(self, on_brightness: float = None, off_brightness: float = 0.0, 
              on_time: float = 0.5, off_time: float = 0.5, count: Optional[int] = None) -> bool:
        """Start blinking pattern"""
        try:
            if on_brightness is None:
                on_brightness = self.max_brightness
            
            self.stop_blink()  # Stop any existing blink
            self.is_blinking = True
            
            def blink_thread():
                blink_count = 0
                while self.is_blinking and (count is None or blink_count < count):
                    if not self.is_blinking:
                        break
                    self.set_brightness(on_brightness)
                    time.sleep(on_time)
                    
                    if not self.is_blinking:
                        break
                    self.set_brightness(off_brightness)
                    time.sleep(off_time)
                    
                    blink_count += 1
                
                self.is_blinking = False
            
            self.blink_thread = threading.Thread(target=blink_thread)
            self.blink_thread.daemon = True
            self.blink_thread.start()
            
            logger.info(f"Started blinking {self.name}: on={on_brightness:.2f}, off={off_brightness:.2f}")
            return True
        except Exception as e:
            logger.error(f"Failed to start blinking {self.name}: {e}")
            return False
    
    def stop_blink(self) -> bool:
        """Stop blinking pattern"""
        if self.is_blinking:
            self.is_blinking = False
            if self.blink_thread and self.blink_thread.is_alive():
                self.blink_thread.join(timeout=1.0)
            logger.debug(f"Stopped blinking {self.name}")
        return True
    
    def pulse(self, min_brightness: float = 0.0, max_brightness: float = None,
              fade_in_time: float = 1.0, fade_out_time: float = 1.0) -> bool:
        """Start pulsing pattern (continuous fade in/out)"""
        try:
            if max_brightness is None:
                max_brightness = self.max_brightness
            
            if GPIO_METHOD == "gpiozero":
                # Use gpiozero's built-in pulse
                self.controller.pulse(fade_in_time=fade_in_time, fade_out_time=fade_out_time,
                                    n=None, background=True)
                logger.info(f"Started pulsing {self.name} (gpiozero built-in)")
                return True
            else:
                # Implement custom pulsing
                self.stop_blink()
                self.is_blinking = True
                
                def pulse_thread():
                    while self.is_blinking:
                        # Fade in
                        steps = int(fade_in_time * 50)
                        for i in range(steps + 1):
                            if not self.is_blinking:
                                break
                            brightness = min_brightness + ((max_brightness - min_brightness) * i / steps)
                            self.set_brightness(brightness)
                            time.sleep(fade_in_time / steps)
                        
                        # Fade out
                        steps = int(fade_out_time * 50)
                        for i in range(steps + 1):
                            if not self.is_blinking:
                                break
                            brightness = max_brightness - ((max_brightness - min_brightness) * i / steps)
                            self.set_brightness(brightness)
                            time.sleep(fade_out_time / steps)
                    
                    self.is_blinking = False
                
                self.blink_thread = threading.Thread(target=pulse_thread)
                self.blink_thread.daemon = True
                self.blink_thread.start()
                
                logger.info(f"Started pulsing {self.name}: {min_brightness:.2f} to {max_brightness:.2f}")
                return True
        except Exception as e:
            logger.error(f"Failed to start pulsing {self.name}: {e}")
            return False
    
    def cleanup(self) -> bool:
        """Cleanup light controller"""
        try:
            self.stop_blink()
            self.off()
            
            if GPIO_METHOD == "gpiozero" and self.controller:
                self.controller.close()
            elif GPIO_METHOD == "RPi.GPIO" and self.controller:
                self.controller.stop()
            
            logger.debug(f"Cleaned up {self.name}")
            return True
        except Exception as e:
            logger.error(f"Failed to cleanup {self.name}: {e}")
            return False

def initialize_lights() -> bool:
    """Initialize the advanced light control system"""
    global lights_initialized, light_states, brightness_levels, LED_CONTROLLERS
    
    try:
        if GPIO_METHOD == "RPi.GPIO":
            GPIO.setmode(GPIO.BCM)
        
        LED_CONTROLLERS = {}
        light_states = {}
        brightness_levels = {}
        
        for name, config in LIGHT_CONFIG.items():
            logger.info(f"Initializing {name}...")
            controller = LightController(name, config)
            if controller.initialize():
                LED_CONTROLLERS[name] = controller
                light_states[name] = False
                brightness_levels[name] = 0.0
                logger.debug(f"Initialized {name} on pin {config['pin']}")
            else:
                logger.warning(f"Failed to initialize {name}")
        
        lights_initialized = True
        logger.info(f"Light system initialized with {len(LED_CONTROLLERS)} lights ({GPIO_METHOD})")
        return True
        
    except Exception as e:
        logger.error(f"Failed to initialize light system: {e}")
        return False

# Convenience functions for easy access
def set_light_brightness(light_name: str, brightness: float) -> bool:
    """Set a specific light's brightness"""
    if not lights_initialized:
        if not initialize_lights():
            logger.error("Failed to initialize lights")
            return False
    
    if light_name not in LED_CONTROLLERS:
        logger.error(f"Unknown light: {light_name}. Available lights: {list(LED_CONTROLLERS.keys())}")
        logger.error(f"LIGHT_CONFIG keys: {list(LIGHT_CONFIG.keys())}")
        logger.error(f"lights_initialized: {lights_initialized}")
        return False
    
    success = LED_CONTROLLERS[light_name].set_brightness(brightness)
    if success:
        light_states[light_name] = brightness > 0
        brightness_levels[light_name] = brightness
    return success

def set_light(light_name: str, state: bool, brightness: Optional[float] = None) -> bool:
    """Set a specific light on or off"""
    if not lights_initialized:
        if not initialize_lights():
            logger.error("Failed to initialize lights")
            return False
    
    if light_name not in LED_CONTROLLERS:
        logger.error(f"Unknown light: {light_name}. Available lights: {list(LED_CONTROLLERS.keys())}")
        return False
    
    controller = LED_CONTROLLERS[light_name]
    if state:
        success = controller.on(brightness)
    else:
        success = controller.off()
    
    if success:
        light_states[light_name] = state
        if state and brightness is not None:
            brightness_levels[light_name] = brightness
    return success

def dim_light(light_name: str, target_brightness: float, duration: float = 2.0) -> bool:
    """Gradually dim a light to target brightness"""
    if not lights_initialized:
        initialize_lights()
    
    if light_name not in LED_CONTROLLERS:
        logger.error(f"Unknown light: {light_name}")
        return False
    
    return LED_CONTROLLERS[light_name].dim(target_brightness, duration)

def blink_light(light_name: str, on_brightness: float = None, off_brightness: float = 0.0,
                on_time: float = 0.5, off_time: float = 0.5, count: Optional[int] = None) -> bool:
    """Start blinking a specific light"""
    if not lights_initialized:
        initialize_lights()
    
    if light_name not in LED_CONTROLLERS:
        logger.error(f"Unknown light: {light_name}")
        return False
    
    return LED_CONTROLLERS[light_name].blink(on_brightness, off_brightness, on_time, off_time, count)

def pulse_light(light_name: str, min_brightness: float = 0.0, max_brightness: float = None,
                fade_in_time: float = 1.0, fade_out_time: float = 1.0) -> bool:
    """Start pulsing a specific light"""
    if not lights_initialized:
        initialize_lights()
    
    if light_name not in LED_CONTROLLERS:
        logger.error(f"Unknown light: {light_name}")
        return False
    
    return LED_CONTROLLERS[light_name].pulse(min_brightness, max_brightness, fade_in_time, fade_out_time)

def stop_light_effects(light_name: str) -> bool:
    """Stop all effects on a specific light"""
    if not lights_initialized:
        initialize_lights()
    
    if light_name not in LED_CONTROLLERS:
        logger.error(f"Unknown light: {light_name}")
        return False
    
    return LED_CONTROLLERS[light_name].stop_blink()

def set_pwm_frequency(light_name: str, frequency_mode: str) -> bool:
    """Change PWM frequency for a light (low/medium/high)"""
    if not lights_initialized:
        initialize_lights()
    
    if light_name not in LED_CONTROLLERS:
        logger.error(f"Unknown light: {light_name}")
        return False
    
    if frequency_mode not in PWM_FREQUENCIES:
        logger.error(f"Invalid frequency mode: {frequency_mode}. Use: {list(PWM_FREQUENCIES.keys())}")
        return False
    
    try:
        controller = LED_CONTROLLERS[light_name]
        current_brightness = controller.current_brightness
        
        # Save current state
        was_on = current_brightness > 0
        
        # Stop current effects and cleanup existing controller
        controller.stop_blink()
        controller.off()
        
        # Properly cleanup the existing controller
        if GPIO_METHOD == "gpiozero" and controller.controller:
            controller.controller.close()
        elif GPIO_METHOD == "RPi.GPIO" and controller.controller:
            controller.controller.stop()
        
        # Reset controller reference
        controller.controller = None
        
        # Update frequency configuration
        LIGHT_CONFIG[light_name]['pwm_freq'] = frequency_mode
        controller.pwm_freq = frequency_mode
        
        # Reinitialize with new frequency
        if controller.initialize():
            # Restore previous brightness if it was on
            if was_on:
                controller.set_brightness(current_brightness)
            
            freq_hz = PWM_FREQUENCIES[frequency_mode]
            logger.info(f"Changed {light_name} PWM frequency to {freq_hz}Hz ({frequency_mode})")
            return True
        else:
            logger.error(f"Failed to reinitialize {light_name} with new frequency")
            return False
            
    except Exception as e:
        logger.error(f"Failed to change PWM frequency for {light_name}: {e}")
        return False

def get_pwm_frequency(light_name: str) -> str:
    """Get current PWM frequency mode for a light"""
    if light_name in LIGHT_CONFIG:
        return LIGHT_CONFIG[light_name]['pwm_freq']
    return None

# Main light control functions
def main_light_on(brightness: float = 1.0) -> bool:
    """Turn main light on"""
    return set_light('main_light', True, brightness)

def main_light_off() -> bool:
    """Turn main light off"""
    return set_light('main_light', False)

def main_light_brightness(brightness: float) -> bool:
    """Set main light brightness (0.0 to 1.0)"""
    return set_light_brightness('main_light', brightness)

def main_light_dim(target_brightness: float, duration: float = 2.0) -> bool:
    """Gradually dim main light to target brightness"""
    return dim_light('main_light', target_brightness, duration)

def main_light_blink(on_brightness: float = 1.0, off_brightness: float = 0.0,
                     on_time: float = 0.5, off_time: float = 0.5, count: int = None) -> bool:
    """Start blinking main light"""
    return blink_light('main_light', on_brightness, off_brightness, on_time, off_time, count)

def main_light_pulse(min_brightness: float = 0.0, max_brightness: float = 1.0,
                     fade_in_time: float = 1.0, fade_out_time: float = 1.0) -> bool:
    """Start pulsing main light"""
    return pulse_light('main_light', min_brightness, max_brightness, fade_in_time, fade_out_time)

def main_light_stop_effects() -> bool:
    """Stop all effects on main light"""
    return stop_light_effects('main_light')

def set_main_light_pwm_frequency(frequency_mode: str) -> bool:
    """Change main light PWM frequency (low/medium/high)"""
    return set_pwm_frequency('main_light', frequency_mode)

def get_main_light_pwm_frequency() -> str:
    """Get current main light PWM frequency mode"""
    return get_pwm_frequency('main_light')

def all_lights_on(brightness: float = None) -> bool:
    """Turn all lights on (alias for main light)"""
    actual_brightness = brightness if brightness is not None else 1.0
    return set_light('main_light', True, actual_brightness)

def all_lights_off() -> bool:
    """Turn all lights off (alias for main light)"""
    return set_light('main_light', False)

def get_light_states() -> Dict[str, Dict]:
    """Get current state of all lights"""
    if not lights_initialized:
        return {}
    
    states = {}
    for name in LIGHT_CONFIG.keys():
        states[name] = {
            'on': light_states.get(name, False),
            'brightness': brightness_levels.get(name, 0.0),
            'max_brightness': LIGHT_CONFIG[name]['max_brightness'],
            'pwm_freq': LIGHT_CONFIG[name]['pwm_freq'],
            'pin': LIGHT_CONFIG[name]['pin'],
            'description': LIGHT_CONFIG[name]['description']
        }
    return states

def cleanup_lights() -> bool:
    """Cleanup light control system"""
    global lights_initialized
    
    try:
        for controller in LED_CONTROLLERS.values():
            controller.cleanup()
        
        if GPIO_METHOD == "RPi.GPIO":
            GPIO.cleanup()
        
        lights_initialized = False
        logger.info("Light system cleanup completed")
        return True
    except Exception as e:
        logger.error(f"Light cleanup error: {e}")
        return False

def test_lights() -> bool:
    """Comprehensive test of all light functions"""
    logger.info("Starting comprehensive light system test...")
    
    if not initialize_lights():
        logger.error("Light initialization failed")
        return False
    
    try:
        # Test 1: Basic on/off control
        logger.info("Test 1: Basic on/off control")
        logger.info("Testing main_light...")
        set_light('main_light', True)
        time.sleep(1)
        set_light('main_light', False)
        time.sleep(0.5)
        
        # Test 2: Brightness control
        logger.info("Test 2: Brightness control")
        logger.info("Testing main_light brightness...")
        for brightness in [0.25, 0.5, 0.75, 1.0]:
            set_light_brightness('main_light', brightness)
            time.sleep(0.8)
        set_light('main_light', False)
        
        # Test 3: Dimming
        logger.info("Test 3: Dimming effects")
        set_light('main_light', True, 0.1)
        dim_light('main_light', 1.0, 2.0)
        time.sleep(2.5)
        dim_light('main_light', 0.0, 1.0)
        time.sleep(1.5)
        
        # Test 4: Blinking patterns
        logger.info("Test 4: Blinking patterns")
        blink_light('main_light', on_brightness=0.8, on_time=0.3, off_time=0.3, count=5)
        time.sleep(3)
        
        # Test 5: Pulsing
        logger.info("Test 5: Pulsing effects")
        pulse_light('main_light', min_brightness=0.1, max_brightness=0.8)
        time.sleep(4)
        stop_light_effects('main_light')
                
        # Test 7: PWM frequency switching demonstration
        logger.info("Test 7: PWM frequency switching demonstration")
        logger.info("Testing different PWM frequencies on main light:")
        
        for freq_mode in ['low', 'medium', 'high']:
            freq_hz = PWM_FREQUENCIES[freq_mode]
            logger.info(f"  Setting main_light to {freq_hz}Hz ({freq_mode} frequency)")
            set_pwm_frequency('main_light', freq_mode)
            set_light_brightness('main_light', 0.7)
            time.sleep(2)
        
        # Reset to medium frequency
        set_pwm_frequency('main_light', 'medium')
        all_lights_off()
        
        # Show final states
        states = get_light_states()
        logger.info("Final light states:")
        for name, state in states.items():
            logger.info(f"  {name}: {state}")
        
        logger.info("Comprehensive light system test completed successfully")
        return True
        
    except Exception as e:
        logger.error(f"Light test failed: {e}")
        return False
    finally:
        cleanup_lights()

if __name__ == "__main__":
    # Run comprehensive light test if this module is executed directly
    test_lights()
