# experiment 01: NVIDIA Rivermark outdoor environment

## goal

load the NVIDIA Rivermark ready-made outdoor scene in Isaac Sim and evaluate
whether it's suitable for Husky A200 visual-inertial SLAM experiments

## setup

- Isaac Sim 6.0.0 (full runtime extracted from container)
- renderer: RayTracedLighting
- GPU: RTX 3090 24GB
- scene source: NVIDIA public S3 asset server

## scene

Rivermark is a suburban commercial area with:
- multi-building shopping plaza with detailed architecture
- large parking lot with lane markings, curbs, sidewalks
- roads with yellow/white lane markings
- grass patches, trees, landscaping
- lampposts, trash cans, benches (street furniture)
- loading docks, utility areas

asset URL:
```
https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Environments/Outdoor/Rivermark/rivermark.usd
```

the scene is ~1000+ files loaded via USD references from S3 at runtime

## spawn location

- robot at (5, 0, 0.5) - parking lot area
- camera at (0, 0, 2) looking along +X
- topdown camera at (0, 0, 80) looking straight down

## commands

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
/opt/isaac-sim-6.0.0/python.sh /workspace/simulation/isaac/scripts/load_rivermark.py
```

## results

### render rate
- 32.1 Hz with RayTracedLighting renderer on RTX 3090
- acceptable for 30 Hz camera simulation

### topdown view
- excellent - all buildings, roads, parking lot, vegetation clearly visible
- mean pixel value: 207 (well-lit)
- rich visual features for SLAM: lane markings, building edges, curbs, grass boundaries

### forward camera view
- **problem**: ground-level camera shows only a bright horizon line
- mean pixel value: 16 (very dim)
- Rivermark materials don't render correctly at ground level with RayTracedLighting
- likely needs PathTracing renderer + longer convergence time, or the scene's
  baked lighting assumes a specific camera setup
- topdown works because material fallback (white/grey) is visible from above

### robot physics
- husky settles to z=0.06m (stable on ground plane)
- root_joint removed, robot free to drive
- physics at 60 Hz

## visual quality for ORB-SLAM3

topdown perspective: excellent - hundreds of distinct features:
- building corners and edges
- parking lot line markings (~200 individual lines)
- curb boundaries
- grass/asphalt texture boundaries
- vehicle-sized objects (trucks, carts)
- road lane markings

**forward perspective:** not usable in current state - scene materials don't
render at ground level with RayTracedLighting. would need:
1. PathTracing renderer (slower, ~5 Hz)
2. proper HDR sky texture loading
3. possibly scene-specific material fixes

## conclusion

Rivermark is a high-quality outdoor scene but has rendering limitations at
ground level with the RayTracedLighting renderer. for the thesis, we have
two options:

1. **use our procedural scene** (cubes + dome sky) - renders correctly at all
   angles, 24 Hz, simple but sufficient for SLAM feature tracking
- use Rivermark with PathTracing - photorealistic but ~5 Hz, too slow
   for real-time SLAM evaluation

recommendation: use the procedural scene for SLAM experiments (it works
reliably) and reference Rivermark as a future improvement when PathTracing
performance allows it

## references

- NVIDIA Rivermark - suburban outdoor scene - [S3 bucket](https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Environments/Outdoor/Rivermark/rivermark.usd)
- **NVIDIA HDR Skies** - sky dome textures - [S3 bucket](https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/NVIDIA/Assets/Skies/)
- **NVIDIA SimReady Assets** - trees, rocks, vegetation - [S3 bucket](https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/NVIDIA/Assets/Vegetation/)
