"""Debounce independent YOLO frames before declaring a target detected."""


class TargetFilter:
    def __init__(self, required_frames=3, min_confidence=0.55):
        if required_frames < 1 or not 0.0 <= min_confidence <= 1.0:
            raise ValueError('Invalid detection requirements')
        self.required_frames = required_frames
        self.min_confidence = min_confidence
        self.target = ''
        self.count = 0
        self.confirmed = False

    def set_target(self, target):
        changed = self.target != target
        if changed:
            self.target = target
            self.count = 0
            self.confirmed = False
        return changed

    def update(self, class_confidence):
        """Accept max confidence per *new frame*, never multiple boxes as frames."""
        conf = class_confidence.get(self.target, 0.0)
        self.count = self.count + 1 if conf >= self.min_confidence else 0
        self.confirmed = self.count >= self.required_frames
        return self.confirmed
