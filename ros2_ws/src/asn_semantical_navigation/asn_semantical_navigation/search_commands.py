"""Small deterministic prompt-to-object-class parser; no LLM or code execution."""

import re


# COCO-compatible aliases. Object classes are checked against the loaded model.
ALIASES = {
    'backpack': ('backpack', 'rucksack', 'school bag', 'schoolbag'),
    'handbag': ('handbag', 'purse'),
    'suitcase': ('suitcase', 'luggage'),
    'cup': ('cup', 'mug', 'coffee cup'),
    'bottle': ('bottle', 'water bottle'),
    'chair': ('chair',),
    'person': ('person', 'hiker', 'someone', 'human'),
    'cell phone': ('cell phone', 'phone', 'mobile phone', 'smartphone'),
    'laptop': ('laptop', 'notebook computer'),
    'book': ('book',),
    'remote': ('remote', 'remote control'),
    'keyboard': ('keyboard',),
    'mouse': ('mouse',),
    'teddy bear': ('teddy bear', 'teddy'),
    'dog': ('dog',),
    'cat': ('cat',),
    'sports ball': ('sports ball', 'ball'),
    'clock': ('clock',),
    'umbrella': ('umbrella',),
}


def parse_target(text, available_classes=None):
    """Return one category or raise ValueError; never guess among two classes.

    The full command is treated as untrusted text. Descriptors such as
    'black' are *not* enforced by the COCO detector.
    """
    if not isinstance(text, str) or len(text) > 300 or not text.strip():
        raise ValueError('A short, nonempty search command is required')
    normalized = ' '.join(re.findall(r"[a-z0-9]+", text.lower()))
    available = (set(available_classes) if available_classes is not None
                 else None)
    detected = set()
    for canonical, terms in ALIASES.items():
        if available is not None and canonical not in available:
            continue
        for alias in terms:
            if re.search(r'(?<!\w)' + re.escape(alias) + r'(?!\w)',
                         normalized):
                detected.add(canonical)
                break
    if not detected:
        raise ValueError('No supported class found in the search command')
    if len(detected) > 1:
        raise ValueError('Ambiguous target: ' + ', '.join(sorted(detected)))
    return detected.pop()
