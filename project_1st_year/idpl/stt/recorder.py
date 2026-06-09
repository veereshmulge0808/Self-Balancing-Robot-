import numpy as np

try:
    import sounddevice as sd
    _sd_error = None
except Exception as exc:
    sd = None
    _sd_error = exc

class VADAudioRecorder:
    def __init__(self, sample_rate=16000, chunk_ms=30,
                 energy_threshold=0.005, silence_duration_ms=800):
        if sd is None:
            raise RuntimeError(f"sounddevice unavailable: {_sd_error}")
        self.sample_rate = sample_rate
        self.chunk_size = sample_rate * chunk_ms // 1000
        self.energy_threshold = energy_threshold
        self.silence_chunks_needed = silence_duration_ms // chunk_ms
        self.state = "WAITING"
        self.buffer = []
        self.pre_buffer = []
        self.silence_counter = 0

    def rms_energy(self, chunk: np.ndarray) -> float:
        return np.sqrt(np.mean(chunk.astype(np.float32)**2))

    def record_utterance(self) -> np.ndarray | None:
        self.state = "WAITING"
        self.buffer = []
        self.pre_buffer = []
        self.silence_counter = 0

        stream = sd.InputStream(samplerate=self.sample_rate, channels=1, 
                                blocksize=self.chunk_size, dtype=np.float32)
        try:
            with stream:
                while True:
                    chunk, overflowed = stream.read(self.chunk_size)
                    chunk = chunk.flatten()
                    rms = self.rms_energy(chunk)

                    if self.state == "WAITING":
                        self.pre_buffer.append(chunk)
                        if len(self.pre_buffer) > 10:
                            self.pre_buffer.pop(0)

                        if rms > self.energy_threshold:
                            print(f"[VAD] Speech detected... (rms: {rms:.4f})")
                            self.state = "SPEAKING"
                            self.buffer.extend(self.pre_buffer)
                            self.buffer.append(chunk)

                    elif self.state == "SPEAKING":
                        self.buffer.append(chunk)
                        if rms < self.energy_threshold:
                            self.state = "SILENCE_AFTER_SPEECH"
                            self.silence_counter = 1
                    
                    elif self.state == "SILENCE_AFTER_SPEECH":
                        self.buffer.append(chunk)
                        if rms > self.energy_threshold:
                            self.state = "SPEAKING"
                            self.silence_counter = 0
                        else:
                            self.silence_counter += 1
                            if self.silence_counter >= self.silence_chunks_needed:
                                self.state = "EMIT"
                    
                    if self.state == "EMIT":
                        print("[VAD] Silence, processing...")
                        audio_data = np.concatenate(self.buffer)
                        return audio_data
        except KeyboardInterrupt:
            return None
