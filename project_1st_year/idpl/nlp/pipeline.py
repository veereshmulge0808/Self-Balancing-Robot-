import re

class NLPPipeline:
    """NLP pipeline for VoxBot self-balancing robot.
    
    Matches voice commands to robot actions based on the firmware's
    parseCommand() function in Voxbot.ino:
      - DRIVE_FORWARD   (linearVelocity = +0.5)
      - DRIVE_BACKWARD  (linearVelocity = -0.5)
      - TURN_LEFT       (angularVelocity = -0.5)
      - TURN_RIGHT      (angularVelocity = +0.5)
      - STOP            (all zeros)
      - SPEED:x.xx      (custom forward speed 0.0-1.0)
    """
    def __init__(self):
        # Ordered by priority — STOP first so "stop" overrides everything
        self._patterns = [
            (re.compile(r"\b(stop|halt|pause|stand\s*still|freeze|brake)\b", re.I), "STOP"),
            (re.compile(r"\b(turn\s*left|rotate\s*left|spin\s*left|go\s*left|left)\b", re.I), "TURN_LEFT"),
            (re.compile(r"\b(turn\s*right|rotate\s*right|spin\s*right|go\s*right|right)\b", re.I), "TURN_RIGHT"),
            (re.compile(r"\b(back|backward|backwards|reverse|go\s*back|move\s*back)\b", re.I), "DRIVE_BACKWARD"),
            (re.compile(r"\b(forward|ahead|go\s*straight|move\s*forward|go\s*forward|advance)\b", re.I), "DRIVE_FORWARD"),
        ]

    def startup(self):
        print("[NLP] Pipeline ready — pattern-matching for robot commands")
        return None

    def infer(self, text: str):
        """Parse voice text into a robot command.
        
        Returns dict with:
          raw_text, intent, confidence, velocity, ble_payload (string)
        """
        raw_text = text.strip() if isinstance(text, str) else ""
        lowered = raw_text.lower()
        command = None
        velocity = 0.5
        confidence = 0.0

        # Match against known patterns
        for pattern, intent in self._patterns:
            if pattern.search(lowered):
                command = intent
                confidence = 0.92
                break

        # Fallback: if no pattern matched
        if command is None:
            command = "STOP"
            confidence = 0.35

        # Extract speed value if mentioned (e.g. "go forward at 0.8")
        speed_match = re.search(
            r"\b([0-9]+(?:\.[0-9]+)?)\s*(?:m/?s|meters?\s*per\s*second|speed|percent)?\b",
            lowered
        )
        if speed_match:
            try:
                val = float(speed_match.group(1))
                if 0.0 <= val <= 1.0:
                    velocity = val
            except ValueError:
                pass

        # Build the BLE payload string matching firmware expectations
        if command.startswith("SPEED"):
            ble_payload = f"SPEED:{velocity:.2f}"
        else:
            ble_payload = command

        return {
            "raw_text": raw_text,
            "intent": command,
            "confidence": confidence,
            "velocity": velocity,
            "ble_payload": ble_payload,
        }
