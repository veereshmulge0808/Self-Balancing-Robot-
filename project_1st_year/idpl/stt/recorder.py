import numpy as np

try:
    import sounddevice as sd
    _sd_error = None
except Exception as exc:
    sd = None
    _sd_error = exc

class VADAudioRecorder:
    """Voice Activity Detection recorder with cancellation support.
    
    Records microphone audio, detects speech onset via RMS energy,
    and returns the audio once silence follows speech.
    
    Key improvements over naive VAD:
      - stop_flag callable: lets the voice thread abort recording
        instantly when the user toggles voice off.
      - Minimum speech duration: prevents false triggers from
        transient noise spikes (mic pops, clicks).
      - Maximum recording duration: prevents infinite blocking
        if the environment is noisy.
    """
    def __init__(self, sample_rate=16000, chunk_ms=30,
                 energy_threshold=0.05, silence_duration_ms=700,
                 min_speech_chunks=15, max_record_seconds=5):
        if sd is None:
            raise RuntimeError(f"sounddevice unavailable: {_sd_error}")
        self.sample_rate = sample_rate
        self.chunk_ms = chunk_ms
        self.chunk_size = sample_rate * chunk_ms // 1000
        self.energy_threshold = energy_threshold
        self.silence_chunks_needed = silence_duration_ms // chunk_ms
        self.min_speech_chunks = min_speech_chunks
        self.max_chunks = int(max_record_seconds * 1000 / chunk_ms)

    def rms_energy(self, chunk: np.ndarray) -> float:
        return np.sqrt(np.mean(chunk.astype(np.float32)**2))

    def record_utterance(self, stop_flag=None) -> np.ndarray | None:
        """Record a single utterance from microphone.
        
        Args:
            stop_flag: callable returning True when recording should abort.
                       Checked every chunk (~30ms). Pass None to never abort.
        
        Returns:
            numpy array of audio samples, or None if cancelled/too short.
        """
        state = "WAITING"
        buffer = []
        pre_buffer = []
        silence_counter = 0
        speech_chunks = 0
        total_chunks = 0

        stream = sd.InputStream(samplerate=self.sample_rate, channels=1, 
                                blocksize=self.chunk_size, dtype=np.float32)
        try:
            with stream:
                while True:
                    # Check cancellation flag every iteration (~30ms)
                    if stop_flag and stop_flag():
                        return None

                    chunk, overflowed = stream.read(self.chunk_size)
                    chunk = chunk.flatten()
                    rms = self.rms_energy(chunk)
                    total_chunks += 1

                    # Safety: bail if recording too long
                    if total_chunks > self.max_chunks:
                        if buffer and speech_chunks >= self.min_speech_chunks:
                            print(f"[VAD] Max duration reached, processing ({speech_chunks} speech chunks)...")
                            return np.concatenate(buffer)
                        return None

                    if state == "WAITING":
                        pre_buffer.append(chunk)
                        if len(pre_buffer) > 10:
                            pre_buffer.pop(0)

                        if rms > self.energy_threshold:
                            print(f"[VAD] Speech onset (rms: {rms:.4f})")
                            state = "SPEAKING"
                            buffer.extend(pre_buffer)
                            buffer.append(chunk)
                            speech_chunks = 1

                    elif state == "SPEAKING":
                        buffer.append(chunk)
                        speech_chunks += 1
                        if rms < self.energy_threshold:
                            state = "SILENCE_AFTER_SPEECH"
                            silence_counter = 1
                    
                    elif state == "SILENCE_AFTER_SPEECH":
                        buffer.append(chunk)
                        if rms > self.energy_threshold:
                            state = "SPEAKING"
                            speech_chunks += 1
                            silence_counter = 0
                        else:
                            silence_counter += 1
                            if silence_counter >= self.silence_chunks_needed:
                                state = "EMIT"
                    
                    if state == "EMIT":
                        # Check minimum speech duration to filter noise
                        if speech_chunks < self.min_speech_chunks:
                            print(f"[VAD] Too short ({speech_chunks} chunks < {self.min_speech_chunks}), ignoring noise.")
                            # Reset and keep listening
                            state = "WAITING"
                            buffer = []
                            pre_buffer = []
                            silence_counter = 0
                            speech_chunks = 0
                            continue

                        duration_ms = len(buffer) * self.chunk_ms
                        print(f"[VAD] Utterance captured: {duration_ms}ms ({speech_chunks} speech chunks)")
                        audio_data = np.concatenate(buffer)
                        return audio_data
        except KeyboardInterrupt:
            return None
        except Exception as exc:
            # PortAudioError, device not found, ALSA errors, etc.
            print(f"[VAD] Stream error: {exc}")
            return None
