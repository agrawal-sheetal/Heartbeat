import pygame
import numpy as np
import math
import noise
import random
import imageio
import os
import subprocess
import time
from datetime import datetime
import serial
import threading
from collections import deque
from pygame import gfxdraw  # For anti-aliased drawing

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Your Heart Rate")

# Colors
COLORS = {
    "green": (0, 255, 0),
    "yellow": (255, 255, 0),
    "orange": (255, 165, 0),
    "red": (255, 0, 0)
}

# Heart rate ranges
HEART_RATE_RANGES = {
    "green": (60, 80),
    "yellow": (81, 100),
    "orange": (101, 120),
    "red": (121, 200)
}

# Transition speeds
TRANSITION_SPEEDS = {
    "green": 2.0,
    "yellow": 1.0,
    "orange": 0.5,
    "red": 0.2
}

# Global variable to store the latest heart rate value
current_heart_rate = 0  # Start with 0 (no finger detected)
heart_rate_lock = threading.Lock()

# Serial connection settings
SERIAL_PORT = "/dev/cu.usbserial-1440"  # Change this to your Arduino's serial port
BAUD_RATE = 115200

# Global variable to control the daemon thread
stop_thread = threading.Event()

def read_heart_rate_from_arduino():
    """Function to continuously read BPM data from Arduino"""
    global current_heart_rate

    try:
        # Connect to the Arduino
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Arduino on {SERIAL_PORT}")
        time.sleep(2)  # Give the connection time to establish

        while not stop_thread.is_set():  # Check the stop flag
            try:
                # Read a line from the serial port
                line = ser.readline().decode('utf-8').strip()
                if line:
                    # Parse the Avg BPM value from the line
                    if line.startswith("IR="):
                        # Check if the line contains "No finger?"
                        if "No finger?" in line:
                            with heart_rate_lock:
                                current_heart_rate = 0  # Set heart rate to 0
                                print("No finger detected. Heart rate set to 0.")
                        else:
                            # Extract Avg BPM value (format: "IR=<irValue>, BPM=<beatsPerMinute>, Avg BPM=<beatAvg>")
                            parts = line.split(", ")
                            if len(parts) >= 3:
                                avg_bpm_part = parts[2]  # "Avg BPM=<beatAvg>"
                                avg_bpm_value = float(avg_bpm_part.split("=")[1])  # Extract the Avg BPM value
                                with heart_rate_lock:
                                    current_heart_rate = avg_bpm_value
                                    print(f"Average Heart Rate: {current_heart_rate:.1f} BPM")

            except Exception as e:
                print(f"Error parsing data: {e}")
                time.sleep(1)

    except Exception as e:
        print(f"Could not connect to Arduino: {e}")
        print("Using simulated heart rate data instead.")
        while not stop_thread.is_set():  # Check the stop flag
            with heart_rate_lock:
                current_heart_rate = random.randint(60, 120)  # Simulate heart rate
            time.sleep(1)  # Simulate delay

# Function to get the current heart rate
def get_heart_rate():
    with heart_rate_lock:
        return current_heart_rate

# Function to get color and transition speed based on heart rate
def get_state(heart_rate):
    if heart_rate == 0:
        return None, 0  # No color or transition if no finger is detected
    if 60 <= heart_rate <= 80:
        return "green", TRANSITION_SPEEDS["green"]
    elif 81 <= heart_rate <= 100:
        return "yellow", TRANSITION_SPEEDS["yellow"]
    elif 101 <= heart_rate <= 120:
        return "orange", TRANSITION_SPEEDS["orange"]
    elif heart_rate >= 121:
        return "red", TRANSITION_SPEEDS["red"]
    else:
        return "green", TRANSITION_SPEEDS["green"]  # Default to green if no range matches

# Function to interpolate between two values
def lerp(start, end, t):
    return start + (end - start) * t

# Function to generate smooth, liquid-like blob points
def generate_liquid_blob_points(position, base_radius, time, noise_scale=0.5, num_points=36):
    points = []
    for i in range(num_points):
        angle = (2 * math.pi * i) / num_points
        noise_value = noise.pnoise2(
            math.cos(angle) * noise_scale,
            math.sin(angle) * noise_scale,
            octaves=4,
            persistence=0.5,
            lacunarity=2.0,
            repeatx=1024,
            repeaty=1024,
            base=int(time)
        )
        radius_offset = base_radius * 0.3 * noise_value
        radius = base_radius + radius_offset
        x = position[0] + radius * math.cos(angle)
        y = position[1] + radius * math.sin(angle)
        points.append((x, y))
    return points

# Function to draw an anti-aliased polygon
def draw_aa_polygon(surface, points, color):
    pygame.gfxdraw.filled_polygon(surface, points, color)
    pygame.gfxdraw.aapolygon(surface, points, color)

# Function to share GIF via iMessage
def share_via_imessage(gif_path):
    try:
        if os.path.exists(gif_path):
            absolute_path = os.path.abspath(gif_path)
            print(f"Preparing to attach GIF: {absolute_path}")

            # First copy the file to clipboard using a temporary script
            temp_script = f'''
            set theFile to POSIX file "{absolute_path}"
            tell application "Finder"
                set the clipboard to theFile as «class furl»
            end tell
            '''

            # Run the script to copy file to clipboard
            subprocess.run(["osascript", "-e", temp_script], check=True)
            time.sleep(0.5)  # Brief pause

            # Now open Messages and create a new message
            messages_script = '''
            tell application "Messages"
                activate
                delay 1
            end tell

            tell application "System Events"
                tell process "Messages"
                    keystroke "n" using command down
                    delay 1
                    # Make sure we're in the message body field, not the To field
                    keystroke tab
                    delay 0.5
                    # Paste the file as an attachment
                    keystroke "v" using command down
                end tell
            end tell
            '''

            # Run the Messages script
            subprocess.run(["osascript", "-e", messages_script], check=True)
            print("Messages opened with new conversation. GIF should be attached to the message body.")

        else:
            print(f"GIF file not found: {absolute_path}")
    except Exception as e:
        print(f"Error sharing via iMessage: {str(e)}")

# Function to save frames as a GIF with transparency
def save_frames_as_gif(frames, filename, fps=20):
    try:
        # Ensure the frames are in the right format for transparency
        processed_frames = []
        for frame in frames:
            # Convert the frame to a format suitable for imageio (RGBA)
            rgba_frame = np.zeros((frame.shape[0], frame.shape[1], 4), dtype=np.uint8)
            rgba_frame[:, :, 0] = frame[:, :, 0]  # Red channel
            rgba_frame[:, :, 1] = frame[:, :, 1]  # Green channel
            rgba_frame[:, :, 2] = frame[:, :, 2]  # Blue channel
            rgba_frame[:, :, 3] = frame[:, :, 3]  # Alpha channel

            # Append the processed frame
            processed_frames.append(rgba_frame)

        # Save as GIF with transparency
        imageio.mimsave(filename, processed_frames, fps=fps, format="GIF", transparency=0)
        print(f"Transparent GIF saved successfully at: {os.path.abspath(filename)}")
        return True
    except Exception as e:
        print(f"Error saving GIF: {e}")
        return False

# Main function with transparent background
def main():
    global stop_thread

    # Start the Arduino reading thread
    arduino_thread = threading.Thread(target=read_heart_rate_from_arduino, daemon=True)
    arduino_thread.start()

    try:
        clock = pygame.time.Clock()
        running = True
        time_counter = 0
        current_radius = 100
        target_radius = 100
        transition_time = 0
        current_color = COLORS["green"]
        frames = []

        # Create a transparent surface
        transparent_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)

        # Variables for pulsing effect
        pulse_phase = 0
        last_heart_rate = get_heart_rate()
        pulse_strength = 0

        # Flag to track whether recording has started
        is_recording = False

        # Variable to track the current state
        current_state = "green"

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Get the current heart rate from Arduino
            heart_rate = get_heart_rate()
            state, transition_duration = get_state(heart_rate)

            if state is None:
                # No finger detected, hide the blob
                current_radius = 0
                target_radius = 0
                current_color = (0, 0, 0, 0)  # Fully transparent
            else:
                # Finger detected, update blob properties
                target_color = COLORS[state]

                # Adjust target radius based on heart rate
                if heart_rate > 100:
                    target_radius = 140
                elif heart_rate > 80:
                    target_radius = 120
                else:
                    target_radius = 100

                # Create pulsing effect based on heart rate
                if heart_rate != last_heart_rate:
                    # When heart rate changes, add a pulse
                    pulse_strength = min(heart_rate / 60, 1.5)  # Stronger pulse for higher heart rates
                    last_heart_rate = heart_rate

                # Pulse effect calculation (simulating heartbeat)
                pulse_phase += 0.1 * (heart_rate / 60)  # Adjust speed based on heart rate
                pulse_factor = math.sin(pulse_phase) * pulse_strength
                pulse_radius = current_radius * (1 + 0.05 * pulse_factor)

                # Reset transition time if the state changes
                if state != current_state:
                    transition_time = 0
                    current_state = state

                # Smooth color and radius transitions
                if abs(current_radius - target_radius) > 1:
                    t = min(transition_time / transition_duration, 1)
                    current_radius = lerp(current_radius, target_radius, t)
                    current_color = (
                        int(lerp(current_color[0], target_color[0], t)),
                        int(lerp(current_color[1], target_color[1], t)),
                        int(lerp(current_color[2], target_color[2], t))
                    )
                    transition_time += clock.get_time() / 1000
                else:
                    transition_time = 0

            # Clear the transparent surface with transparent pixels
            transparent_surface.fill((0, 0, 0, 0))

            # Draw the blob on the transparent surface (if finger is detected)
            if state is not None:
                blob_position = (WIDTH // 2, HEIGHT // 2)
                blob_points = generate_liquid_blob_points(blob_position, pulse_radius, time_counter)

                # Add alpha value to the color for transparency support
                blob_color = (current_color[0], current_color[1], current_color[2], 255)  # Fully opaque blob
                draw_aa_polygon(transparent_surface, blob_points, blob_color)  # Anti-aliased drawing

            # Blit the transparent surface to the main screen
            screen.fill((0, 0, 0, 0))  # Fill with transparent black
            screen.blit(transparent_surface, (0, 0))

            # Capture the frame only if recording has started
            if is_recording or heart_rate > 0:
                if not is_recording:
                    print("Recording started (BPM > 0 detected).")
                    is_recording = True

                # Capture the frame - need to handle transparency for the GIF
                frame = pygame.surfarray.array3d(screen)  # Shape: (WIDTH, HEIGHT, 3)
                frame = np.transpose(frame, (1, 0, 2))  # Transpose to (HEIGHT, WIDTH, 3)

                # Create an alpha channel - all pixels are fully opaque where the blob is
                # and transparent elsewhere
                alpha = pygame.surfarray.array_alpha(transparent_surface)  # Shape: (WIDTH, HEIGHT)
                alpha = np.transpose(alpha)  # Transpose to (HEIGHT, WIDTH)

                # Create RGBA frame
                rgba_frame = np.zeros((HEIGHT, WIDTH, 4), dtype=np.uint8)
                rgba_frame[:, :, 0] = frame[:, :, 0]  # Red channel
                rgba_frame[:, :, 1] = frame[:, :, 1]  # Green channel
                rgba_frame[:, :, 2] = frame[:, :, 2]  # Blue channel
                rgba_frame[:, :, 3] = alpha[:, :]  # Alpha channel

                frames.append(rgba_frame)

            pygame.display.flip()
            time_counter += 0.02
            clock.tick(30)

        save_directory = "/Users/sheetalagrawal/Downloads/hearbeat_visualization"
        if not os.path.exists(save_directory):
            os.makedirs(save_directory)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        gif_filename = f"heartbeat_visualization_{timestamp}.gif"
        gif_path = os.path.join(save_directory, gif_filename)

        if is_recording and save_frames_as_gif(frames, gif_path, fps=10):
            # Only if the GIF was saved successfully, share it
            share_via_imessage(gif_path)

    finally:
        # Signal the daemon thread to stop
        stop_thread.set()
        pygame.quit()

if __name__ == "__main__":
    main()