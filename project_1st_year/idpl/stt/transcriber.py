import numpy as np

try:
    from faster_whisper import WhisperModel
    _whisper_error = None
except Exception as exc:
    WhisperModel = None
    _whisper_error = exc

class WhisperTranscriber:
    def __init__(self, model_size="tiny.en", device="cpu", compute_type="int8"):
        if WhisperModel is None:
            raise RuntimeError(f"Whisper model unavailable: {_whisper_error}")
        self.model = WhisperModel(model_size, device=device, compute_type=compute_type)

    def transcribe(self, audio: np.ndarray, sample_rate=16000) -> str:
        segments, info = self.model.transcribe(audio, beam_size=5, language="en")
        
        text = "".join([segment.text for segment in segments])
        text = text.strip().lower()
        
        if not text or all(char in ".,?! " for char in text):
            return ""
            
        return text
