#!/usr/bin/env python3
"""
KML to ArduPilot Waypoint Converter v3
Enhanced with camera controls, resolution settings, buffer zones, and optimized square wave patterns
"""

import xml.etree.ElementTree as ET
import math
import sys
import json
from typing import List, Tuple, Optional, Dict
import re

class KMLToWaypointV3:
    def __init__(self,
                 altitude: int = 10,
                 spacing: float = 0.00005,
                 buffer_distance: float = 0.00002,
                 camera_settings: Optional[Dict] = None):
        """
        Initialize enhanced converter

        Args:
            altitude: Flight altitude in meters (integer only)
            spacing: Distance between parallel lines in decimal degrees (default: ~5m)
            buffer_distance: Buffer distance from polygon boundary in decimal degrees (~2m)
            camera_settings: Dict with camera configuration
        """
        self.altitude = altitude
        self.spacing = spacing
        self.buffer_distance = buffer_distance

        # Default camera settings
        self.camera_settings = {
            'trigger_mode': 1,          # 0=disable, 1=neutral, 2=servo, 3=relay
            'trigger_distance': 5,      # Distance between photos in meters
            'gimbal_tilt': -90,         # Camera tilt angle (-90 = straight down)
            'gimbal_mode': 2,           # 0=retract, 1=neutral, 2=mavlink_targeting, 3=rc_targeting
            'resolution_width': 4000,   # Camera resolution width
            'resolution_height': 3000,  # Camera resolution height
            'overlap_percent': 80,      # Photo overlap percentage
            'sidelap_percent': 60       # Side overlap percentage
        }

        if camera_settings:
            self.camera_settings.update(camera_settings)

    def calculate_optimal_spacing(self) -> float:
        """
        Calculate optimal line spacing based on camera settings and desired overlap
        """
        # Ground sample distance calculation (approximate)
        sensor_width = 23.5  # mm (typical APS-C sensor)
        focal_length = 24    # mm (typical wide angle)

        # Calculate ground coverage width at given altitude
        ground_width = (sensor_width * self.altitude) / focal_length

        # Calculate spacing for desired sidelap
        sidelap_factor = (100 - self.camera_settings['sidelap_percent']) / 100
        optimal_spacing = ground_width * sidelap_factor

        # Convert to decimal degrees (rough approximation)
        optimal_spacing_degrees = optimal_spacing / 111000

        return max(optimal_spacing_degrees, 0.00001)  # Minimum 1m spacing

    def parse_kml(self, kml_file: str) -> List[Tuple[float, float]]:
        """
        Parse KML file and extract polygon coordinates
        """
        try:
            tree = ET.parse(kml_file)
            root = tree.getroot()

            # Handle namespace
            namespace = {'kml': 'http://www.opengis.net/kml/2.2'}
            if root.tag.startswith('{'):
                namespace_match = re.match(r'\{([^}]+)\}', root.tag)
                if namespace_match:
                    namespace = {'kml': namespace_match.group(1)}

            # Find coordinates in the KML
            coordinates_elem = None
            paths_to_try = [
                './/kml:Polygon/kml:outerBoundaryIs/kml:LinearRing/kml:coordinates',
                './/kml:Polygon/kml:exterior/kml:LinearRing/kml:coordinates',
                './/kml:coordinates'
            ]

            for path in paths_to_try:
                try:
                    coordinates_elem = root.find(path, namespace)
                    if coordinates_elem is not None:
                        break
                except:
                    continue

            if coordinates_elem is None:
                for path in ['.//Polygon//coordinates', './/coordinates']:
                    coordinates_elem = root.find(path)
                    if coordinates_elem is not None:
                        break

            if coordinates_elem is None:
                raise ValueError("Could not find coordinates in KML file")

            # Parse coordinates
            coords_text = coordinates_elem.text.strip()
            coordinates = []

            for line in coords_text.split():
                if line.strip():
                    parts = line.strip().split(',')
                    if len(parts) >= 2:
                        lon = float(parts[0])
                        lat = float(parts[1])
                        coordinates.append((lat, lon))

            if len(coordinates) < 3:
                raise ValueError("Need at least 3 coordinates to define a polygon")

            print(f"Parsed {len(coordinates)} boundary points from KML")
            return coordinates

        except Exception as e:
            print(f"Error parsing KML file: {e}")
            return []

    def create_buffer_polygon(self, polygon: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Create an inward buffer polygon to keep drone away from boundaries
        """
        if len(polygon) < 3:
            return polygon

        # Simple inward buffer by moving each point toward the centroid
        # Calculate centroid
        center_lat = sum(p[0] for p in polygon) / len(polygon)
        center_lon = sum(p[1] for p in polygon) / len(polygon)

        buffered_polygon = []
        for lat, lon in polygon:
            # Vector from point to center
            to_center_lat = center_lat - lat
            to_center_lon = center_lon - lon

            # Normalize and scale by buffer distance
            distance = math.sqrt(to_center_lat**2 + to_center_lon**2)
            if distance > 0:
                buffer_lat = lat + (to_center_lat / distance) * self.buffer_distance
                buffer_lon = lon + (to_center_lon / distance) * self.buffer_distance
                buffered_polygon.append((buffer_lat, buffer_lon))
            else:
                buffered_polygon.append((lat, lon))

        print(f"Applied {self.buffer_distance * 111000:.1f}m buffer zone from boundaries")
        return buffered_polygon

    def point_in_polygon(self, point: Tuple[float, float], polygon: List[Tuple[float, float]]) -> bool:
        """Ray casting algorithm for point-in-polygon test"""
        lat, lon = point
        n = len(polygon)
        inside = False

        p1lat, p1lon = polygon[0]
        for i in range(1, n + 1):
            p2lat, p2lon = polygon[i % n]
            if lon > min(p1lon, p2lon):
                if lon <= max(p1lon, p2lon):
                    if lat <= max(p1lat, p2lat):
                        if p1lon != p2lon:
                            xinters = (lon - p1lon) * (p2lat - p1lat) / (p2lon - p1lon) + p1lat
                        if p1lat == p2lat or lat <= xinters:
                            inside = not inside
            p1lat, p1lon = p2lat, p2lon

        return inside

    def get_polygon_bounds(self, polygon: List[Tuple[float, float]]) -> Tuple[float, float, float, float]:
        """Get bounding box of polygon"""
        lats = [p[0] for p in polygon]
        lons = [p[1] for p in polygon]
        return min(lats), max(lats), min(lons), max(lons)

    def find_polygon_intersections(self, lat: float, polygon: List[Tuple[float, float]]) -> List[float]:
        """Find longitude intersections where horizontal line crosses polygon boundary"""
        intersections = []
        n = len(polygon)
        tolerance = 1e-10

        for i in range(n):
            p1_lat, p1_lon = polygon[i]
            p2_lat, p2_lon = polygon[(i + 1) % n]

            # Check if the edge crosses the horizontal line
            if abs(p1_lat - p2_lat) < tolerance:  # Horizontal edge
                if abs(p1_lat - lat) < tolerance:  # Edge lies on the scan line
                    intersections.extend([p1_lon, p2_lon])
            else:
                # Check if scan line crosses the edge
                if ((p1_lat <= lat < p2_lat) or (p2_lat <= lat < p1_lat)):
                    # Calculate intersection longitude
                    lon_intersect = p1_lon + (lat - p1_lat) * (p2_lon - p1_lon) / (p2_lat - p1_lat)
                    intersections.append(lon_intersect)

        # Remove duplicates and sort
        intersections = sorted(list(set(intersections)))

        # Group intersections into pairs for entry/exit points
        paired_intersections = []
        i = 0
        while i < len(intersections):
            if i + 1 < len(intersections):
                paired_intersections.extend([intersections[i], intersections[i + 1]])
                i += 2
            else:
                i += 1

        return paired_intersections

    def generate_square_wave_lawnmower(self, polygon: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Generate optimized square wave lawnmower pattern with proper turns
        """
        if len(polygon) < 3:
            return []

        # Apply buffer to keep away from boundaries
        buffered_polygon = self.create_buffer_polygon(polygon)
        min_lat, max_lat, min_lon, max_lon = self.get_polygon_bounds(buffered_polygon)

        # Calculate optimal spacing if using camera settings
        if self.camera_settings.get('auto_spacing', False):
            self.spacing = self.calculate_optimal_spacing()

        waypoints = []
        current_lat = min_lat + self.spacing / 2  # Start slightly inside
        going_east = True
        line_count = 0

        print(f"Generating optimized square wave lawnmower pattern...")
        print(f"Survey area bounds: lat {min_lat:.6f} to {max_lat:.6f}, lon {min_lon:.6f} to {max_lon:.6f}")
        print(f"Line spacing: {self.spacing:.6f} degrees ({self.spacing * 111000:.1f}m)")
        print(f"Buffer distance: {self.buffer_distance * 111000:.1f}m from boundaries")

        while current_lat <= max_lat - self.spacing / 2:
            intersections = self.find_polygon_intersections(current_lat, buffered_polygon)

            if len(intersections) >= 2:
                # Process intersection pairs
                for i in range(0, len(intersections) - 1, 2):
                    left_lon = intersections[i]
                    right_lon = intersections[i + 1]

                    # Ensure we have valid segment
                    if abs(right_lon - left_lon) > self.spacing / 10:
                        if going_east:
                            waypoints.append((current_lat, left_lon))
                            waypoints.append((current_lat, right_lon))
                        else:
                            waypoints.append((current_lat, right_lon))
                            waypoints.append((current_lat, left_lon))

                        line_count += 1

            current_lat += self.spacing
            going_east = not going_east

        print(f"Generated {len(waypoints)} waypoints across {line_count} scan lines")
        return waypoints

    def add_camera_commands(self, waypoints: List[Tuple[float, float]]) -> List[Dict]:
        """
        Add camera trigger and gimbal commands to waypoints
        """
        enhanced_waypoints = []

        for i, (lat, lon) in enumerate(waypoints):
            waypoint = {
                'lat': lat,
                'lon': lon,
                'alt': self.altitude,
                'commands': []
            }

            # Add camera trigger command if enabled
            if self.camera_settings['trigger_mode'] > 0:
                # Camera trigger distance command
                if i == 0:  # First waypoint
                    waypoint['commands'].append({
                        'type': 'CAM_TRIGG_DIST',
                        'distance': self.camera_settings['trigger_distance']
                    })

                # Gimbal control
                waypoint['commands'].append({
                    'type': 'DO_MOUNT_CONTROL',
                    'pitch': self.camera_settings['gimbal_tilt'],
                    'roll': 0,
                    'yaw': 0,
                    'save': 0
                })

            enhanced_waypoints.append(waypoint)

        return enhanced_waypoints

    def create_waypoint_file(self, waypoints: List[Tuple[float, float]], output_file: str,
                           home_point: Optional[Tuple[float, float]] = None):
        """
        Create enhanced ArduPilot waypoint file with camera controls
        """
        if not waypoints:
            print("No waypoints to write")
            return

        if home_point is None:
            home_point = waypoints[0]

        try:
            with open(output_file, 'w') as f:
                # Write header
                f.write("QGC WPL 110\n")

                waypoint_index = 0

                # Home point
                f.write(f"{waypoint_index}\t0\t0\t16\t0.000000\t0.000000\t0.000000\t0.000000\t"
                       f"{home_point[0]:.6f}\t{home_point[1]:.6f}\t0.100000\t1\n")
                waypoint_index += 1

                # Takeoff command
                f.write(f"{waypoint_index}\t0\t3\t22\t0.000000\t0.000000\t0.000000\t0.000000\t"
                       f"0.000000\t0.000000\t{self.altitude}.000000\t1\n")
                waypoint_index += 1

                # Camera setup commands
                if self.camera_settings['trigger_mode'] > 0:
                    # Set camera trigger distance
                    f.write(f"{waypoint_index}\t0\t3\t206\t{self.camera_settings['trigger_distance']:.1f}\t"
                           f"0.000000\t0.000000\t0.000000\t0.000000\t0.000000\t0.000000\t1\n")
                    waypoint_index += 1

                    # Set gimbal mode
                    f.write(f"{waypoint_index}\t0\t3\t221\t{self.camera_settings['gimbal_tilt']:.1f}\t"
                           f"0.000000\t0.000000\t0.000000\t0.000000\t0.000000\t0.000000\t1\n")
                    waypoint_index += 1

                # Survey waypoints
                for lat, lon in waypoints:
                    f.write(f"{waypoint_index}\t0\t3\t16\t0.000000\t0.000000\t0.000000\t0.000000\t"
                           f"{lat:.6f}\t{lon:.6f}\t{self.altitude}.000000\t1\n")
                    waypoint_index += 1

                # Stop camera trigger
                if self.camera_settings['trigger_mode'] > 0:
                    f.write(f"{waypoint_index}\t0\t3\t206\t0.000000\t0.000000\t0.000000\t0.000000\t"
                           f"0.000000\t0.000000\t0.000000\t1\n")
                    waypoint_index += 1

                # RTL command
                f.write(f"{waypoint_index}\t0\t0\t20\t0.000000\t0.000000\t0.000000\t0.000000\t"
                       f"0.000000\t0.000000\t0.000000\t1\n")

            print(f"Enhanced waypoint file created: {output_file}")
            print(f"Total mission items: {waypoint_index + 1}")
            self.print_mission_summary(waypoints)

        except Exception as e:
            print(f"Error writing waypoint file: {e}")

    def print_mission_summary(self, waypoints: List[Tuple[float, float]]):
        """Print detailed mission summary"""
        if not waypoints:
            return

        print("\n=== MISSION SUMMARY ===")
        print(f"Survey waypoints: {len(waypoints)}")
        print(f"Flight altitude: {self.altitude}m AGL")
        print(f"Line spacing: {self.spacing * 111000:.1f}m")
        print(f"Buffer from boundaries: {self.buffer_distance * 111000:.1f}m")

        if self.camera_settings['trigger_mode'] > 0:
            print(f"Camera trigger: ENABLED")
            print(f"  Trigger distance: {self.camera_settings['trigger_distance']}m")
            print(f"  Gimbal tilt: {self.camera_settings['gimbal_tilt']}°")
            print(f"  Expected overlap: {self.camera_settings['overlap_percent']}%")
            print(f"  Expected sidelap: {self.camera_settings['sidelap_percent']}%")
        else:
            print(f"Camera trigger: DISABLED")

        # Calculate approximate mission distance
        total_distance = 0
        for i in range(len(waypoints) - 1):
            lat1, lon1 = waypoints[i]
            lat2, lon2 = waypoints[i + 1]
            # Approximate distance calculation
            dlat = (lat2 - lat1) * 111000
            dlon = (lon2 - lon1) * 111000 * math.cos(math.radians(lat1))
            total_distance += math.sqrt(dlat**2 + dlon**2)

        print(f"Estimated flight distance: {total_distance:.0f}m")
        print(f"Pattern type: Square wave lawnmower with buffer zone")
        print("========================\n")

def main():
    """Enhanced main function with camera and configuration options"""
    if len(sys.argv) < 3:
        print("KML to ArduPilot Waypoint Converter v3")
        print("Usage: python kml_to_waypoint_v3.py <input.kml> <altitude> [options]")
        print("\nRequired arguments:")
        print("  input.kml     Path to KML polygon file")
        print("  altitude      Flight altitude in meters (integer)")
        print("\nOptional arguments:")
        print("  --spacing METERS        Line spacing in meters (default: auto-calculated)")
        print("  --buffer METERS         Buffer distance from boundaries (default: 2m)")
        print("  --no-camera            Disable camera triggers")
        print("  --trigger-dist METERS   Distance between photos (default: 5m)")
        print("  --gimbal-tilt DEGREES   Camera tilt angle (default: -90°)")
        print("  --overlap PERCENT       Photo overlap percentage (default: 80%)")
        print("  --sidelap PERCENT       Side overlap percentage (default: 60%)")
        print("\nExample:")
        print("  python kml_to_waypoint_v3.py survey_area.kml 50 --spacing 8 --buffer 5")
        return

    input_file = sys.argv[1]

    try:
        altitude = int(sys.argv[2])
        if altitude <= 0:
            print("Error: Altitude must be a positive integer")
            return
    except ValueError:
        print("Error: Altitude must be an integer")
        return

    # Parse optional arguments
    spacing = None  # Will auto-calculate if None
    buffer_distance = 0.00002  # ~2m default
    camera_enabled = True
    trigger_distance = 5
    gimbal_tilt = -90
    overlap_percent = 80
    sidelap_percent = 60

    i = 3
    while i < len(sys.argv):
        if sys.argv[i] == '--spacing' and i + 1 < len(sys.argv):
            spacing = float(sys.argv[i + 1]) / 111000  # Convert meters to degrees
            i += 2
        elif sys.argv[i] == '--buffer' and i + 1 < len(sys.argv):
            buffer_distance = float(sys.argv[i + 1]) / 111000  # Convert meters to degrees
            i += 2
        elif sys.argv[i] == '--no-camera':
            camera_enabled = False
            i += 1
        elif sys.argv[i] == '--trigger-dist' and i + 1 < len(sys.argv):
            trigger_distance = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--gimbal-tilt' and i + 1 < len(sys.argv):
            gimbal_tilt = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--overlap' and i + 1 < len(sys.argv):
            overlap_percent = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--sidelap' and i + 1 < len(sys.argv):
            sidelap_percent = float(sys.argv[i + 1])
            i += 2
        else:
            i += 1

    # Set default spacing if not provided
    if spacing is None:
        spacing = 0.00005  # ~5m default

    # Camera settings
    camera_settings = {
        'trigger_mode': 1 if camera_enabled else 0,
        'trigger_distance': trigger_distance,
        'gimbal_tilt': gimbal_tilt,
        'gimbal_mode': 2,
        'overlap_percent': overlap_percent,
        'sidelap_percent': sidelap_percent,
        'auto_spacing': spacing is None
    }

    # Output filename
    output_file = "waypoints_v3.txt"

    # Create converter
    converter = KMLToWaypointV3(
        altitude=altitude,
        spacing=spacing,
        buffer_distance=buffer_distance,
        camera_settings=camera_settings
    )

    # Parse KML file
    print(f"Parsing KML file: {input_file}")
    polygon = converter.parse_kml(input_file)

    if not polygon:
        print("Failed to parse KML file or no valid polygon found")
        return

    # Generate square wave lawnmower pattern
    waypoints = converter.generate_square_wave_lawnmower(polygon)

    if not waypoints:
        print("No waypoints generated within polygon boundary")
        return

    # Create waypoint file
    converter.create_waypoint_file(waypoints, output_file)

    print(f"Conversion complete! Output saved as: {output_file}")

if __name__ == "__main__":
    main()
