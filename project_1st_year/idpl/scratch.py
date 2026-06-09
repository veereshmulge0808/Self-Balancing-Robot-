import sys
import numpy as np
from stt.recorder import VADAudioRecorder
from stt.transcriber import WhisperTranscriber

def test():
    print("Loading transcriber...")
    transcriber = WhisperTranscriber()
    print("Loading recorder...")
    recorder = VADAudioRecorder()
    print("Speak now!")
    audio = recorder.record_utterance()
    if audio is not None:
        print(f"Recorded shape: {audio.shape}, max: {np.max(audio)}, min: {np.min(audio)}")
        try:
            text = transcriber.transcribe(audio)
            print(f"Transcription: '{text}'")
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    test()
