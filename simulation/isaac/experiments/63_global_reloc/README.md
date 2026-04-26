# Exp 63 - Global Relocalisation Fallback

## результат

| Area | Exp 62 | Exp 63 |
|---|---|---|
| Global relocalisation fallback | - | **added** |
| Relocalisation trigger | - | matcher silent >= 20 s |
| Relocalisation candidate pool | - | all landmarks with heading within 90° |
| Relocalisation strict PnP | - | MIN_INLIERS 10->18, REPROJ 2.0->1.5 px |
| Consistency check | reject if shift > 5 m | **skipped when relocating** |
