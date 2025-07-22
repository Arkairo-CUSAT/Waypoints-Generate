
# KML to ArduPilot Waypoint Converter v8

## 🚀 What's New in v8?

- **Smart Corner Detection**: Identifies polygon corners (top-right, top-left, etc.) for optimal flight planning
- **Optimized Start/End**: Flight pattern starts from the top-right and ends near the top-left, minimizing return distance to home
- **Minimal Return Distance**: Pattern direction and ordering are chosen to reduce total flight time and battery use
- **Enhanced Ordering**: Considers home position and polygon geometry for best waypoint sequence
- **Detailed Mission Summary**: Reports corner analysis, pattern direction, and distance optimizations

---

A powerful Python tool that converts KML polygon files into ArduPilot waypoint missions with advanced camera controls, customizable flight patterns, home position optimization, smart waypoint sequencing, and experimental trapezoid optimization.

## Features

- **🆕 Trapezoid Optimization**: Automatically analyzes polygon shape to fly parallel to the longest side for maximum efficiency
- **🆕 Home-Aware Finishing**: Intelligently ends flight pattern at the side closest to home position
- **Home Position Optimization**: Set custom home coordinates (--home-lat, --home-lon) for optimized flight planning
- **Smart Waypoint Sequencing**: Automatically optimizes waypoint order to start and end closest to home position
- **Intelligent Flight Patterns**: Auto-select optimal direction or choose between vertical/horizontal lawnmower patterns
- **Fence Padding**: Configurable safety buffer to keep drone inside polygon boundaries
- **Camera Integration**: Automatic camera trigger and gimbal control for aerial photography
- **Optimized Spacing**: Auto-calculated line spacing based on camera settings for optimal photo overlap
- **Buffer Zones**: Inward buffer to maintain safe distance from boundaries
- **Square Wave Pattern**: Efficient flight path with minimal turns
- **Distance Calculations**: Accurate GPS distance calculations using Haversine formula
- **Enhanced Mission Summary**: Detailed flight statistics including total mission distance and home-to-waypoint distances

## Requirements

- Python 3.6+
- Standard Python libraries (xml.etree.ElementTree, math, sys, json, re, typing)

## Installation

1. Clone or download the script:
```bash
git clone <repository-url>
# or download kml_to_wayp_v6.py directly
```

2. Make the script executable:
```bash
chmod +x kml_to_wayp_v6.py
```

## Usage

### Basic Syntax
```bash
python kml_to_wayp_v6.py <input.kml> <altitude> [options]
```

### Required Arguments
- `input.kml` - Path to your KML polygon file
- `altitude` - Flight altitude in meters (integer only)

### Optional Arguments

| Option | Description | Default |
|--------|-------------|---------|
| `--spacing METERS` | Line spacing between flight paths | Auto-calculated |
| `--fence-padding METERS` | Distance to stay inside fence boundaries | 2m |
| `--pattern auto\|vertical\|horizontal` | Flight pattern direction | auto (trapezoid optimized) |
| `--home-lat LATITUDE` | Home position latitude (for optimization) | None |
| `--home-lon LONGITUDE` | Home position longitude (for optimization) | None |
| `--no-camera` | Disable camera triggers | Camera enabled |
| `--trigger-dist METERS` | Distance between photos | 5m |
| `--gimbal-tilt DEGREES` | Camera tilt angle | -90° (straight down) |
| `--overlap PERCENT` | Photo overlap percentage | 80% |
| `--sidelap PERCENT` | Side overlap percentage | 60% |

## Examples


### Basic Usage (v8)
```bash
# Smart corner-optimized survey at 50m altitude
python kml_to_wayp_v8.py survey_area.kml 50

# Optimized mission with home position (recommended)
python kml_to_wayp_v8.py survey_area.kml 50 --home-lat 12.345678 --home-lon 78.901234
```


### Advanced Usage (v8)
```bash
# High-resolution survey with corner optimization and home positioning
python kml_to_wayp_v8.py detailed_survey.kml 25 --spacing 3 --fence-padding 3 --trigger-dist 3 --home-lat 12.345678 --home-lon 78.901234

# Custom camera settings with corner-optimized pattern
python kml_to_wayp_v8.py photo_mission.kml 60 --gimbal-tilt -45 --overlap 90 --sidelap 70 --home-lat 12.345678 --home-lon 78.901234
```


## Flight Patterns

### v8: Corner-Optimized Pattern (Default)
- **Direction**: Starts at top-right, ends at top-left (minimizes return distance)
- **Algorithm**: Flies parallel to the longest side, but orders scan lines and waypoints for best home proximity
- **Optimization**: Uses polygon corner analysis and home position for pattern direction and ordering
- **Advantages**: Maximum efficiency, minimal battery use, best for all survey areas
- **Best for**: All polygons, especially when home position is set

### Vertical Pattern
- **Direction**: North-South flight lines
- **Advantages**: Fewer turns for most survey areas, more efficient for wide areas
- **Best for**: Wide areas, standard aerial photography

### Horizontal Pattern
- **Direction**: East-West flight lines  
- **Advantages**: Better for tall, narrow areas
- **Best for**: Linear features, specific geometric requirements


## Output

The script generates:
1. **Waypoint file**: `waypoints_v8.txt` in ArduPilot format (saved to `~/ardu-sim/`)
2. **Mission summary**: Detailed flight statistics including corner analysis and pattern optimization
3. **Progress information**: Real-time updates with geometry and corner detection

### Sample Output (v8)
```
Home position set to: 12.345678, 78.901234
Parsing KML file: survey_area.kml
Parsed 8 boundary points from KML
Fence padding: 2.0m
Using corner-optimized parallel flight pattern...
Applied 2.0m buffer zone from boundaries
Analyzing polygon sides for optimal flight direction...
Side 0: length=245.3m, bearing=15.2°
Side 1: length=180.7m, bearing=78.4°
Side 2: length=198.1m, bearing=25.8°
Side 3: length=425.6m, bearing=82.1°
Longest side: 3 (425.6m, bearing=82.1°)
Polygon corners identified:
  top_right: (40.126789, -74.983210)
  top_left: (40.126789, -74.987654)
  bottom_left: (40.123456, -74.987654)
  bottom_right: (40.123456, -74.983210)
Generating flight lines parallel to longest side (bearing: 82.1°)
Optimizing waypoint ordering for minimal return distance...
Pattern point closest to top-right corner: first_start at (40.126789, -74.983210)
Starting pattern from start of first line
Distance from pattern start to home: 52.0m
Distance from pattern end to home: 31.0m
Generated 28 waypoints across 7 scan lines

=== MISSION SUMMARY ===
Home position: 12.345678, 78.901234
Survey waypoints: 28
Flight altitude: 50m AGL
Line spacing: 5.0m
Buffer from boundaries: 2.0m
Camera trigger: ENABLED
  Trigger distance: 5m
  Gimbal tilt: -90°
  Expected overlap: 80%
  Expected sidelap: 60%
Distance from home to first waypoint: 52m
Estimated survey flight distance: 920m
Distance from last waypoint to home: 31m
Total mission distance: 1003m
Pattern type: Corner-optimized parallel flight with smart ordering
```

## KML File Requirements

Your KML file should contain:
- A single polygon with at least 3 vertices
- Coordinates in decimal degrees (WGS84)
- Standard KML polygon structure

### Supported KML Formats
```xml
<!-- Google Earth KML -->
<Polygon>
  <outerBoundaryIs>
    <LinearRing>
      <coordinates>...</coordinates>
    </LinearRing>
  </outerBoundaryIs>
</Polygon>

<!-- Simple KML -->
<Polygon>
  <exterior>
    <LinearRing>
      <coordinates>...</coordinates>
    </LinearRing>
  </exterior>
</Polygon>
```

## Safety Features

### Trapezoid Optimization (NEW in v6)
Intelligent polygon analysis for optimal flight patterns:
- **Geometry Analysis**: Automatically measures all polygon sides and determines longest edge
- **Direction Selection**: Chooses flight direction parallel to longest side for efficiency
- **Home-Aware Ending**: Analyzes which polygon side is closest to home position
- **Pattern Reversal**: Reverses flight pattern if needed to end closer to home
- **Efficiency Gains**: Reduces overall flight time and battery consumption

### Home Position Optimization
Version 5+ intelligent waypoint sequencing based on a specified home position:
- **Custom Home Coordinates**: Set with `--home-lat` and `--home-lon` parameters
- **Automatic Optimization**: Finds the closest waypoint to home and optimizes mission start
- **Pattern Reversal**: May reverse entire flight pattern to minimize travel distance
- **Distance Minimization**: Reduces both pre-mission and post-mission flight distances

### Fence Padding
- Keeps drone safely inside polygon boundaries
- Configurable distance (default: 2 meters)
- Applied as inward buffer from all boundaries

### Buffer Algorithm
The script uses a centroid-based inward buffer:
1. Calculates polygon centroid
2. Moves each vertex toward center by specified distance
3. Creates smaller internal polygon for flight path

## Camera Settings

### Automatic Spacing Calculation
When spacing is not specified, the script calculates optimal line spacing based on:
- Camera sensor size (23.5mm APS-C default)
- Focal length (24mm wide angle default)  
- Flight altitude
- Desired photo overlap percentages

### Camera Commands Generated
- `CAM_TRIGG_DIST`: Sets photo trigger distance
- `DO_MOUNT_CONTROL`: Controls gimbal positioning
- Automatic camera start/stop commands

## Mission File Format

Output files use QGroundControl waypoint format (.wayplan compatible):
- Header: `QGC WPL 110`
- Home point setup
- Takeoff command
- Camera configuration commands
- Survey waypoints (NAV_WAYPOINT)
- Camera stop command
- Return to Launch (RTL)

## Troubleshooting

### Common Issues

1. **"No valid polygon found"**
   - Check KML file format
   - Ensure polygon has at least 3 vertices
   - Verify coordinates are in decimal degrees

2. **"No waypoints generated"**
   - Reduce fence padding
   - Check polygon size vs. spacing settings
   - Verify polygon is not too small

3. **Too many/few waypoints**
   - Adjust `--spacing` parameter
   - Modify polygon size
   - Check altitude settings for auto-spacing

### File Permissions
```bash
# If you get permission errors:
chmod +x kml_to_wayp_v6.py
chmod 755 /home/arjun/ardu-sim/  # Output directory
```

## Technical Details

### Coordinate System
- Input: WGS84 decimal degrees
- Processing: Maintains WGS84 throughout
- Output: ArduPilot standard format

### Distance Calculations
- Approximate conversion: 1 degree ≈ 111,000 meters
- Accounts for latitude variation in longitude calculations
- Uses Euclidean distance approximation for short distances

### Pattern Generation Algorithm
1. Parse KML polygon
2. **NEW**: Analyze polygon geometry to find longest side and optimal flight direction
3. Apply inward buffer (fence padding)
4. Calculate polygon bounding box
5. Generate parallel scan lines in optimal direction
6. Find polygon intersections
7. **NEW**: Optimize pattern to end at side closest to home position
8. Optimize waypoint sequence based on home position
9. Create alternating flight pattern
10. Add camera and navigation commands

## Version History

### v6.0 Features (NEW - Experimental)
- **🚀 Trapezoid Optimization**: Analyzes polygon geometry to fly parallel to longest side
- **🎯 Home-Aware Finishing**: Ends flight pattern at polygon side closest to home position
- **📐 Geometry Analysis**: Automatic side length calculation and bearing analysis
- **⚡ Efficiency Improvements**: Reduced flight time through intelligent pattern selection
- **🔄 Smart Pattern Reversal**: Reverses entire pattern when beneficial for home proximity
- **📊 Enhanced Logging**: Detailed geometry analysis output during mission planning

### v5.0 Features
- **Home position optimization**: Set custom home coordinates with --home-lat and --home-lon
- **Smart waypoint sequencing**: Automatically finds closest waypoints to home position
- **Enhanced mission summary**: Shows total mission distance and home-to-waypoint distances
- **Improved flight patterns**: Pattern reversal and optimization for efficient missions
- **Better distance calculations**: Haversine formula for accurate GPS distances
- **Enhanced output format**: waypoints_v5.txt saved to ~/ardu-sim/ directory

### v3.0 Features (Previous)
- Vertical flight patterns (new default)
- Runtime fence padding configuration
- Pattern selection (vertical/horizontal)
- Enhanced camera controls
- Improved buffer algorithm
- Better error handling and user feedback

## Contributing

Feel free to submit issues, feature requests, or improvements to enhance the tool's functionality.

## License

MIT License, reserved to the Arkairo Foundation , CUSAT

**Note**: Always verify generated waypoint files before flight. Test in simulation first and ensure compliance with local aviation regulations.
