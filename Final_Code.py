import lgpio_rpi5 as GPIO
import time
import math
from statistics import median
import numpy as np
import sounddevice as sd
from scipy.signal import butter, lfilter
import pygame
import random
import csv
import os
from datetime import datetime


# =========================
# ---- USER PARAMETERS ----
# =========================

# Stepper pins (BCM)
#A1, B1, A2, B2 = 5, 26, 6, 13 # 5purple,26blue,6green,13yellow

coil_pins = [5, 26, 6, 13]  # A1, B1, A1, B2 # Setup GPIO pins
GPIO.setmode(GPIO.BCM)

for pin in coil_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

# Rangefinder pins (BCM)
GPIO.setmode(GPIO.BCM)
TRIG, ECHO = 17, 27 #17top brown, 27top yellow
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Motion behavior
SLOW_DIR = "CW"           # slow scan direction
FAST_DIR = "CCW"          # “escape” direction (opposite of SLOW_DIR)
SLOW_DELAY_S = 0.008      # slow step delay (~125 steps/sec)
FAST_DELAY_S = 0.0015     # fast step delay (~666 steps/sec)
USE_HALF_STEP = True      # smoother slow scan; keep True

FULL_STEPS_PER_REV = 200  # 1.8°/step typical
HALF_STEPS_PER_REV = FULL_STEPS_PER_REV * 2

# Range / logic thresholds
DETECT_ON_M  = .10        # trigger at or below this
DETECT_OFF_M = .13        # resume scan when above this (hysteresis)
MEDIAN_SAMPLES = 3        # samples per reading
SAMPLE_GAP_S = 0.06       # ~60 ms per HC-SR04 spec
TEMP_C = 20.0             # for speed of sound calc

# =========================
# ---- STEPPER SETUP ------
# =========================

#GPIO.setmode(GPIO.BCM)

#for p in COILS:
    #GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

# Half-step (8) and full-step (4) sequences
full_step_seq = [ # Full-step sequence
    [1, 0, 0, 1],  # Step 1
    [1, 1, 0, 0],  # Step 2
    [0, 1, 1, 0],  # Step 3
    [0, 0, 1, 1],  # Step 4
]

half_step_seq = [ #Half-step sequence
    [1, 0, 0, 0],  # Step 1
    [1, 1, 0, 0],  # Step 2
    [0, 1, 0, 0],  # Step 3
    [0, 1, 1, 0],  # Step 4
    [0, 0, 1, 0],  # Step 5
    [0, 0, 1, 1],  # Step 6
    [0, 0, 0, 1],  # Step 7
    [1, 0, 0, 1],  # Step 8
]

def step_motor(sequence, steps, delay=.01, clockwise=True):
    seq = sequence if clockwise else sequence[::-1] #chage direction if necessary
    total_steps = 0 #keep track fo steps
    start_time = time.time() #keep track of time
    for _ in range(steps): 
        for pattern in seq: #go trough each sequence
            for pin, val in zip(coil_pins, pattern):
                GPIO.output(pin, val)
            time.sleep(delay)
            total_steps += 1
    end_time = time.time()

def deenergize():
    for p in coil_pins:
        GPIO.output(p, GPIO.LOW)

# =========================
# ---- RANGEFINDER --------
# =========================

def measure_distance(temp_c=20.0): 

    v = 331.4 + 0.6 * temp_c     # speed of sound (m/s) formula

    GPIO.output(TRIG, False) #make sure trigger is off at beginning
    time.sleep(0.05)

    GPIO.output(TRIG, True) #send short signal
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0: #wait for signal to come back
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start #calculate time is takes for signal to come back
    distance_m = (v * pulse_duration) / 2 #calculate distance by dividing time it takes to come back
    return distance_m #return distance

# =========================
# ---- FREQUENCY FINDER --------
# =========================

def bandpass(data, low=80, high=300, fs=44100, order=3):
    b, a = butter(order, [low / (fs / 2), high / (fs / 2)], btype='band')
    return lfilter(b, a, data)

def get_pitch(samples, fs):
    window = samples * np.hanning(len(samples))
    spectrum = np.abs(np.fft.rfft(window))
    freqs = np.fft.rfftfreq(len(window), 1 / fs)
    return freqs[np.argmax(spectrum)]


# =========================
# ---- MAIN STATE LOOP ----
# =========================

print("Starting: slow scan until ≤ 1.0 m; then fast 2 rev reverse and stop. Ctrl+C to exit.")

fs = 48000  # Sample rate
N = 1024  # Block size
DEVICE_INDEX = 1

position = 12

CSV_FILE = "range_pitch_log.csv"
log_buffer = []  # holds rows until "reposition" happens


pygame.mixer.init() #initialize soudn player

with sd.InputStream(device=DEVICE_INDEX, channels=1, samplerate=fs, blocksize=N) as stream:
    print("\nSpeak into your mic. Ctrl+C to stop.\n")
    try:
        while True:
            # Measure the distance
            d = measure_distance(22.0)  # Use median distance
            print(f"Distance: {d:.2f} meters")

            # Get the audio data from the microphone
            audio_chunk, _ = stream.read(N)
            samples = audio_chunk[:, 0]  # Use the left channel
            filtered = bandpass(samples)  # Apply bandpass filter
            pitch = get_pitch(filtered, fs)  # Get the pitch in Hz
            print(f"Pitch: {pitch:.1f} Hz")

            #buffer data
            log_buffer.append({
                "timestamp": datetime.now().isoformat(timespec="seconds"),
                "distance_m": float(d),
                "pitch_hz": float(pitch),
                "position": int(position),
            })

            #Check fi distance meets condition
            if d < .10:
                step_motor(half_step_seq, 8, 0.01, clockwise=False)
                position += 1
                # Now check if both distance and pitch meet the condition
                if d < .10 and 400 <= pitch <= 600:
                    n = random.randint(1, 12)*8
                    print(f"Object detected close and pitch {pitch} Hz in range, reversing {n} steps.")
                    step_motor(full_step_seq, n, 0.01, clockwise=True)
                    position -= n
                    message = position % 6

                    #Play respective sound
                    if message == 0:
                        sound = pygame.mixer.Sound("1.wav")
                        sound.play()
                        time.sleep(sound.get_length())
                    
                    elif message == 1:
                        sound = pygame.mixer.Sound("2.wav")
                        sound.play()
                        time.sleep(sound.get_length())
                
                    elif message == 2:
                        sound = pygame.mixer.Sound("3.wav")
                        sound.play()
                        time.sleep(sound.get_length())

                    elif message == 3:
                        sound = pygame.mixer.Sound("4.wav")
                        sound.play()
                        time.sleep(sound.get_length())

                    elif message == 4:
                        sound = pygame.mixer.Sound("5.wav")
                        sound.play()
                        time.sleep(sound.get_length())

                    elif message == 5:
                        sound = pygame.mixer.Sound("6.wav")
                        sound.play()
                        time.sleep(sound.get_length())

            #reposition if more than 3 rotations and save .csv
            elif position >= 48:
                print(f"Repositioning")

                if log_buffer:
                    file_exists = os.path.isfile(CSV_FILE)
                    with open(CSV_FILE, "a", newline="") as f:
                        writer = csv.DictWriter(f, fieldnames=["timestamp", "distance_m", "pitch_hz", "position"])
                        if not file_exists:
                            writer.writeheader()
                        writer.writerows(log_buffer)

                    print(f"Saved {len(log_buffer)} samples to {CSV_FILE}")
                    log_buffer.clear() 

                step_motor(full_step_seq, position, 0.01, clockwise=True)
                
    except KeyboardInterrupt:
        print("\nExiting by user request.")
    finally:
        deenergize()
        GPIO.cleanup()
