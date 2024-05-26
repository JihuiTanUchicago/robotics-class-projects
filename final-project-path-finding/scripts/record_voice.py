import sounddevice as sd
import numpy as np
from pydub import AudioSegment
from pydub.playback import play
import io

# Function to record audio from the microphone
def record_audio(duration=5, samplerate=44100):
    print("User Input Recording...")
    recording = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype=np.int16)
    sd.wait()  # Wait until recording is finished
    print("Recording finished.")
    return recording

# Function to save numpy array as MP3 file
def save_as_mp3(audio_data, samplerate=44100, filename="output.mp3"):
    # Convert numpy array to audio segment
    audio_segment = AudioSegment(
        audio_data.tobytes(), 
        frame_rate=samplerate,
        sample_width=audio_data.dtype.itemsize, 
        channels=1
    )
    # Export as MP3
    audio_segment.export(filename, format="mp3")
    print(f"Audio saved as {filename}")

# Main function to capture audio and save it as MP3
def record_voice():
    duration = 5  # Record for 5 seconds
    samplerate = 44100

    audio_data = record_audio(duration, samplerate)
    save_as_mp3(audio_data, samplerate)


