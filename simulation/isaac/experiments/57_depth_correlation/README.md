# exp 57 - visual T&R v3 (depth cross-correlation channel - negative result)

Add a second anchor channel (depth-based cross-correlation against the   
teach occupancy map) to complement the ORB-based visual_landmark_matcher
from exp 55/56.  Hypothesis: geometry-only matching would fire where
visual matching fails (cone zones, feature-poor forest) and close the
drift gap that ORB alone cannot.

## результат

- channel does NOT fire

After multiple treshold relaxations and a dilation tune, the
correlation matcher **failed to publish a single anchor** in the full
repeat run.

Why: in a dense forest, the teach occupancy grid and the current-frame
scan both contain +-100-200 cells each, spread over a broad tree canopy.
Cross-correlation across the +-3 m search window produces a **flat
score surface** - most offsets give 70-90 % hit fraction (every offset
