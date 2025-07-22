# KML to ArduPilot Waypoint Converter v3

A powerful Python tool that converts KML polygon files into ArduPilot waypoint missions with advanced camera controls, customizable flight patterns, and safety features.

## Features

- **Dual Flight Patterns**: Choose between vertical (north-south) or horizontal (east-west) lawnmower patterns
- **Fence Padding**: Configurable safety buffer to keep drone inside polygon boundaries
- **Camera Integration**: Automatic camera trigger and gimbal control for aerial photography
- **Optimized Spacing**: Auto-calculated line spacing based on camera settings for optimal photo overlap
- **Buffer Zones**: Inward buffer to maintain safe distance from boundaries
- **Square Wave Pattern**: Efficient flight path with minimal turns

## Requirements

- Python 3.6+
- Standard Python libraries (xml.etree.ElementTree, math, sys, json, re, typing)

## Installation

1. Clone or download the script:
```bash
git clone <repository-url>
# or download old_v3.py directly
```

2. Make the script executable:
```bash
chmod +x old_v3.py
```

## Usage

### Basic Syntax
```bash
python old_v3.py <input.kml> <altitude> [options]
```

### Required Arguments
- `input.kml` - Path to your KML polygon file
- `altitude` - Flight altitude in meters (integer only)

### Optional Arguments

| Option | Description | Default |
|--------|-------------|---------|
| `--spacing METERS` | Line spacing between flight paths | Auto-calculated |
| `--fence-padding METERS` | Distance to stay inside fence boundaries | 2m |
| `--pattern vertical\|horizontal` | Flight pattern direction | vertical |
| `--no-camera` | Disable camera triggers | Camera enabled |
| `--trigger-dist METERS` | Distance between photos | 5m |
| `--gimbal-tilt DEGREES` | Camera tilt angle | -90° (straight down) |
| `--overlap PERCENT` | Photo overlap percentage | 80% |
| `--sidelap PERCENT` | Side overlap percentage | 60% |

## Examples

### Basic Usage
```bash
# Simple vertical survey at 50m altitude
python old_v3.py survey_area.kml 50

# Horizontal pattern with custom fence padding
python old_v3.py survey_area.kml 30 --pattern horizontal --fence-padding 5
```

### Advanced Usage
```bash
# High-resolution survey with tight spacing
python old_v3.py detailed_survey.kml 25 --spacing 3 --fence-padding 3 --trigger-dist 3

# No camera survey flight
python old_v3.py inspection_area.kml 40 --no-camera --fence-padding 10

# Custom camera settings
python old_v3.py photo_mission.kml 60 --gimbal-tilt -45 --overlap 90 --sidelap 70
```

## Flight Patterns

### Vertical Pattern (Default - Recommended)
- **Direction**: North-South flight lines
- **Advantages**: Fewer turns for most survey areas, more efficient
- **Best for**: Wide areas, standard aerial photography

### Horizontal Pattern
- **Direction**: East-West flight lines  
- **Advantages**: Better for tall, narrow areas
- **Best for**: Linear features, specific geometric requirements

## Output

The script generates:
1. **Waypoint file**: `waypoints_v3.txt` in ArduPilot format
2. **Mission summary**: Detailed flight statistics
3. **Progress information**: Real-time processing updates

### Sample Output
```
Parsing KML file: survey_area.kml
Parsed 8 boundary points from KML
Applied 2.0m buffer zone from boundaries
Generating optimized vertical lawnmower pattern...
Survey area bounds: lat 40.123456 to 40.126789, lon -74.987654 to -74.983210
Line spacing: 0.000045 degrees (5.0m)
Buffer distance: 2.0m from boundaries
Generated 24 waypoints across 6 scan lines

=== MISSION SUMMARY ===
Survey waypoints: 24
Flight altitude: 50m AGL
Line spacing: 5.0m
Fence padding: 2.0m from boundaries
Camera trigger: ENABLED
  Trigger distance: 5m
  Gimbal tilt: -90°
  Expected overlap: 80%
  Expected sidelap: 60%
Estimated flight distance: 850m
Pattern type: Optimized lawnmower with fence padding
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
chmod +x old_v3.py
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
2. Apply inward buffer (fence padding)
3. Calculate polygon bounding box
4. Generate parallel scan lines
5. Find polygon intersections
6. Create alternating flight pattern
7. Add camera and navigation commands

## Version History

### v3.0 Features
- Vertical flight patterns (new default)
- Runtime fence padding configuration
- Pattern selection (vertical/horizontal)
- Enhanced camera controls
- Improved buffer algorithm
- Better error handling and user feedback

## Contributing

Feel free to submit issues, feature requests, or improvements to enhance the tool's functionality.

## License

[Add your license information here]

---

**Note**: Always verify generated waypoint files before flight. Test in simulation first and ensure compliance with local aviation regulations.
