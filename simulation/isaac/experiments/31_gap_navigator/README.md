# exp 31: Gap-Based Navigator with Forest Thinning

Замінити VFH-секторний підхід (exp 30) на gap-based навігатор який знаходить
реальні проходи між деревами з урахуванням ширини Husky. Також перевірити
вплив щільності лісу на навігацію.

## результат

| Metric | Value |
|---|---|
| Objects | 743 (195 pine + 113 oak + 357 shrub + 28 rock + 40 fallen + 10 other) |
| Max anchor reached | **10/96** |
| Distance (GT) | +-20m |
| Stuck point | (-80, -22) - shrubs at 1.2m from both sides |
| Cause | Geometric bottleneck: gap <1.3m (Husky needs 1.27m) |
