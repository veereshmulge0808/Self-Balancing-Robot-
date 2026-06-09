import numpy as np
from faster_whisper import WhisperModel
import soundfile as sf
import pyttsx3

# Generate speech
engine = pyttsx3.init()
engine.save_to_file("go forward fast", "test.wav")
engine.runAndWait()

# Read the audio
data, samplerate = sf.read("test.wav")
if len(data.shape) > 1:
    data = data.mean(axis=1) # to mono

# Resample if needed (pyttsx3 might save at 22050 or 44100)
import librosa
data = librosa.resample(data, orig_sr=samplerate, target_sr=16000)

print(f"Audio shape: {data.shape}, max: {np.max(data)}")

# Normalize like in recorder.py
max_val = np.max(np.abs(data))
if max_val > 0:
    data = data / max_val

model = WhisperModel("tiny.en", device="cpu", compute_type="int8")
segments, info = model.transcribe(data, beam_size=5, language="en")
text = "".join([s.text for s in segments])
print("Transcription:", text)

