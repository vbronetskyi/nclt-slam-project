# exp 42: Encoder+Compass Localization

Replace GT (ground truth) localization from exp 41 with realistic sensor-based
localization that matches what a real Husky A200 provides. Prove the
teach-and-repeat + Nav2 system works without perfect pose information.

## результат

Encoder+compass localization matches GT performance on all tests.

| Localization | No obstacles | With obstacles |
|---|---|---|
| GT (exp 41) | 43/43 (100%) | 41/43 (95%) |
| **Encoder+Compass** | **43/43 (100%), 396s** | **41/43 (95%), 461s** |

Max position drift: 0.8m over 183m (0.4%). Heading error bounded at +-3° RMS.
