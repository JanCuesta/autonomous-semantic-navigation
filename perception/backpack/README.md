# YOLO Backpack Detection — V1 and V2

Fine-tuned YOLO object detectors for backpack recognition
as part of the Autonomous Semantic Navigation Robot project.

## Models

- V1: `../../models/backpack_v1.pt`
- V2: `../../models/backpack_v2.pt`
- Detection class: backpack

## Training Results

Final recorded validation epoch:

| Metric | V1 | V2 |
|---|---:|---:|
| Epochs completed | 50 | 29 |
| Precision | 91.89% | 96.59% |
| Recall | 54.90% | 55.48% |
| mAP@50 | 59.20% | 60.55% |
| mAP@50-95 | 50.33% | 49.18% |

These results come from training-run validation logs,
not a separately verified independent test evaluation.

## Demonstration Videos

- [Watch V1 demonstration](demos/v1_demo.mp4)
- [Watch V2 demonstration](demos/v2_demo.mp4)

Both videos are 46.3-second backpack-detection demonstrations.

Some dataset frames appear to originate from the same
recording series as the demonstrations. These videos are
qualitative examples, not independent generalization tests.

## Contents

- `dataset/`: labeled YOLO dataset
- `training/v1/`: V1 configuration, plots and metrics
- `training/v2/`: V2 configuration, plots and metrics
- `demos/`: compressed demonstration videos

Original training configurations retain historical local paths.
Replace those paths when reproducing training on another machine.
