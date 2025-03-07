import pigpio
import time

SERVO_PIN = 18  # GPIO 18 (Physical Pin 12)
DELAY = 0.02  # Delay between movements for smooth transition
STEP = 10  # Pulse width step for smooth movement

# Connect to pigpio daemon
pi = pigpio.pi()

if not pi.connected:
    print("Error: Unable to connect to pigpio daemon. Run 'sudo pigpiod'.")
    exit(1)

# Function to move servo smoothly
def move_servo(target_pulse):
    current_pulse = pi.get_servo_pulsewidth(SERVO_PIN)
    if current_pulse == 0:
        current_pulse = 1500  # Default to center if not set

    while abs(target_pulse - current_pulse) > STEP:
        if target_pulse > current_pulse:
            current_pulse += STEP
        else:
            current_pulse -= STEP

        pi.set_servo_pulsewidth(SERVO_PIN, current_pulse)
        print(f"Moving to pulse: {current_pulse}")
        time.sleep(DELAY)

    pi.set_servo_pulsewidth(SERVO_PIN, target_pulse)  # Ensure it reaches the exact target
    print(f"Reached target: {target_pulse}")

try:
    while True:
        print("Scanning from 0° to 180°...")
        move_servo(2500)  # Move to 180° (max pulse)

        print("Scanning from 180° back to 0°...")
        move_servo(500)  # Move back to 0° (min pulse)

except KeyboardInterrupt:
    print("Stopping servo")
    pi.set_servo_pulsewidth(SERVO_PIN, 0)  # Stop PWM signal
    pi.stop()
