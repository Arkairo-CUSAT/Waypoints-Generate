#!/usr/bin/env python3
"""
KML to ArduPilot Waypoint Converter v8
Enhanced with smart corner detection and optimized waypoint ordering for minimal return distance
"""

import xml.etree.ElementTree as ET
import math
import sys
import json
import os
from typing import List, Tuple, Optional, Dict
import re

class KMLToWaypointV8:
    def __init__(self,
                 altitude: int = 10,
                 spacing: float = 0.00005,
                 buffer_distance: float = 0.00002,
                 camera_settings: Optional[Dict] = None,
                 home_position: Optional[Tuple[float, float]] = None):
        """
        Initialize enhanced converter with smart corner detection and optimized waypoint ordering

        Args:
            altitude: Flight altitude in meters (integer only)
            spacing: Distance between parallel lines in decimal degrees (default: ~5m)
            buffer_distance: Buffer distance from polygon boundary in decimal degrees (~2m)
            camera_settings: Dict with camera configuration
            home_position: Home position as (lat, lon) tuple
        """
        self.altitude = altitude
        self.spacing = spacing
        self.buffer_distance = buffer_distance
        self.home_position = home_position

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

    def calculate_distance(self, point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
        """
        Calculate approximate distance between two lat/lon points in meters using Haversine formula
        """
        lat1, lon1 = point1
        lat2, lon2 = point2
        
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        # Haversine formula
        a = (math.sin(dlat/2)**2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        # Earth's radius in meters
        R = 6371000
        return R * c

    def find_longest_side_with_bearing(self, polygon: List[Tuple[float, float]]) -> Tuple[int, float, float, Tuple[float, float], Tuple[float, float]]:
        """
        Find the longest side of the polygon and return its bearing
        Returns: (side_index, length_meters, bearing_degrees, start_point, end_point)
        """
        if len(polygon) < 3:
            return 0, 0, 0, polygon[0], polygon[1] if len(polygon) > 1 else polygon[0]
        
        max_length = 0
        longest_side_index = 0
        best_bearing = 0
        best_start = polygon[0]
        best_end = polygon[1] if len(polygon) > 1 else polygon[0]
        
        print("Analyzing polygon sides for optimal flight direction...")
        
        for i in range(len(polygon)):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % len(polygon)]
            
            length = self.calculate_distance(p1, p2)
            
            # Calculate true bearing from p1 to p2
            lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
            lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
            
            dlon = lon2 - lon1
            
            # Calculate bearing using the forward azimuth formula
            y = math.sin(dlon) * math.cos(lat2)
            x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
            
            bearing = math.atan2(y, x)
            bearing = math.degrees(bearing)
            bearing = (bearing + 360) % 360  # Normalize to 0-360
            
            print(f"Side {i}: length={length:.1f}m, bearing={bearing:.1f}°")
            
            if length > max_length:
                max_length = length
                longest_side_index = i
                best_bearing = bearing
                best_start = p1
                best_end = p2
        
        print(f"Longest side: {longest_side_index} ({max_length:.1f}m, bearing={best_bearing:.1f}°)")
        return longest_side_index, max_length, best_bearing, best_start, best_end

    def find_polygon_corners(self, polygon: List[Tuple[float, float]]) -> Dict[str, Tuple[float, float]]:
        """
        Find the four corners of the polygon (top-right, top-left, bottom-left, bottom-right)
        """
        if len(polygon) < 3:
            return {}
        
        # Get bounds
        min_lat, max_lat, min_lon, max_lon = self.get_polygon_bounds(polygon)
        
        # Find actual polygon points closest to each corner
        corners = {}
        
        # Top-right corner (max lat, max lon)
        top_right_dist = float('inf')
        top_right_point = None
        
        # Top-left corner (max lat, min lon)
        top_left_dist = float('inf')
        top_left_point = None
        
        # Bottom-left corner (min lat, min lon)
        bottom_left_dist = float('inf')
        bottom_left_point = None
        
        # Bottom-right corner (min lat, max lon)
        bottom_right_dist = float('inf')
        bottom_right_point = None
        
        for point in polygon:
            lat, lon = point
            
            # Distance to theoretical corners
            tr_dist = math.sqrt((lat - max_lat)**2 + (lon - max_lon)**2)
            tl_dist = math.sqrt((lat - max_lat)**2 + (lon - min_lon)**2)
            bl_dist = math.sqrt((lat - min_lat)**2 + (lon - min_lon)**2)
            br_dist = math.sqrt((lat - min_lat)**2 + (lon - max_lon)**2)
            
            if tr_dist < top_right_dist:
                top_right_dist = tr_dist
                top_right_point = point
                
            if tl_dist < top_left_dist:
                top_left_dist = tl_dist
                top_left_point = point
                
            if bl_dist < bottom_left_dist:
                bottom_left_dist = bl_dist
                bottom_left_point = point
                
            if br_dist < bottom_right_dist:
                bottom_right_dist = br_dist
                bottom_right_point = point
        
        corners = {
            'top_right': top_right_point,
            'top_left': top_left_point,
            'bottom_left': bottom_left_point,
            'bottom_right': bottom_right_point
        }
        
        print("Polygon corners identified:")
        for corner_name, corner_point in corners.items():
            if corner_point:
                print(f"  {corner_name}: ({corner_point[0]:.6f}, {corner_point[1]:.6f})")
        
        return corners

    def generate_parallel_lines_to_longest_side(self, polygon: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Generate flight lines truly parallel to the longest side of the polygon with optimized ordering
        """
        if len(polygon) < 3:
            return []

        # Apply buffer to keep away from boundaries
        buffered_polygon = self.create_buffer_polygon(polygon)
        
        # Find longest side and its bearing
        side_index, side_length, bearing, side_start, side_end = self.find_longest_side_with_bearing(buffered_polygon)
        
        # Find polygon corners for optimization
        corners = self.find_polygon_corners(buffered_polygon)
        
        print(f"Generating flight lines parallel to longest side (bearing: {bearing:.1f}°)")
        
        # Calculate perpendicular direction for spacing
        perp_bearing = (bearing + 90) % 360
        
        # Get polygon bounds
        min_lat, max_lat, min_lon, max_lon = self.get_polygon_bounds(buffered_polygon)
        
        # Calculate the center of the polygon
        center_lat = (min_lat + max_lat) / 2
        center_lon = (min_lon + max_lon) / 2
        
        # Convert spacing from degrees to meters for calculation
        spacing_meters = self.spacing * 111000
        
        # Calculate how far we need to extend the pattern
        diagonal_distance = self.calculate_distance((min_lat, min_lon), (max_lat, max_lon))
        max_extend = diagonal_distance / 2
        
        # Collect all valid lines with their metadata
        all_lines = []
        
        # Start from center and work outward in both directions
        for direction in [-1, 1]:  # negative and positive directions
            offset_distance = 0
            
            while offset_distance <= max_extend:
                if offset_distance == 0 and direction == -1:
                    offset_distance += spacing_meters
                    continue  # Skip center line for negative direction (we'll do it in positive)
                
                # Calculate offset point from center along perpendicular direction
                offset_lat, offset_lon = self.point_at_bearing_and_distance(
                    center_lat, center_lon, perp_bearing, offset_distance * direction
                )
                
                # Generate line through this point parallel to longest side
                line_intersections = self.get_line_intersections_only(
                    offset_lat, offset_lon, bearing, buffered_polygon
                )
                
                if len(line_intersections) >= 2:
                    # Sort intersections by distance from the bearing start point to ensure consistent ordering
                    bearing_start_point = (offset_lat, offset_lon)
                    line_intersections.sort(key=lambda p: self.calculate_distance(bearing_start_point, p))
                    
                    # Store line with its offset distance and intersection points
                    all_lines.append({
                        'offset': offset_distance * direction,
                        'intersections': line_intersections,
                        'start_point': line_intersections[0],
                        'end_point': line_intersections[-1]
                    })
                
                offset_distance += spacing_meters
        
        # Sort lines by their offset distance
        all_lines.sort(key=lambda x: x['offset'])
        
        # Optimize line ordering and direction based on home position and corner analysis
        waypoints = self.optimize_line_ordering_for_corners(all_lines, corners, bearing)
        
        print(f"Generated {len(waypoints)} waypoints across {len(all_lines)} scan lines")
        
        return waypoints

    def optimize_line_ordering_for_corners(self, all_lines: List[Dict], corners: Dict, bearing: float) -> List[Tuple[float, float]]:
        """
        Optimize the line ordering to start from top-right area and end near top-left for minimal return distance
        """
        if not all_lines:
            return []
        
        print("Optimizing waypoint ordering for minimal return distance...")
        
        # Determine if we should start from the "higher" or "lower" numbered lines
        # based on where the corners are relative to the scan direction
        
        # First, determine the scan direction relative to cardinal directions
        # Normalize bearing to standard compass directions
        norm_bearing = bearing % 360
        
        # Get the approximate positions of line endpoints
        first_line = all_lines[0]
        last_line = all_lines[-1]
        
        # For each line, determine which endpoint is "right" and which is "left"
        # relative to the scan bearing
        waypoints = []
        
        # Strategy: Start from the end of the pattern that's closest to top-right corner
        # and work towards top-left corner
        
        if self.home_position and corners.get('top_right') and corners.get('top_left'):
            # Calculate distances from home to both pattern ends
            
            # Check first line endpoints
            first_line_start = first_line['start_point']
            first_line_end = first_line['end_point']
            
            # Check last line endpoints  
            last_line_start = last_line['start_point']
            last_line_end = last_line['end_point']
            
            # Calculate distances to top-right corner for pattern direction decision
            tr_corner = corners['top_right']
            tl_corner = corners['top_left']
            
            # Find which pattern end is closer to top-right
            pattern_starts = [
                (first_line_start, "first_start"),
                (first_line_end, "first_end"),
                (last_line_start, "last_start"), 
                (last_line_end, "last_end")
            ]
            
            # Find closest to top-right corner
            closest_to_tr = min(pattern_starts, 
                               key=lambda x: self.calculate_distance(x[0], tr_corner))
            
            print(f"Pattern point closest to top-right corner: {closest_to_tr[1]} at {closest_to_tr[0]}")
            
            # Determine line order and direction based on this
            if closest_to_tr[1].startswith("first"):
                # Start from first line
                line_order = range(len(all_lines))
                if closest_to_tr[1] == "first_end":
                    # Start from end of first line
                    start_from_end = True
                else:
                    start_from_end = False
            else:
                # Start from last line
                line_order = range(len(all_lines) - 1, -1, -1)
                if closest_to_tr[1] == "last_end":
                    start_from_end = True
                else:
                    start_from_end = False
            
            print(f"Starting pattern from {'end' if start_from_end else 'start'} of {'first' if closest_to_tr[1].startswith('first') else 'last'} line")
            
        else:
            # Fallback: use original ordering
            line_order = range(len(all_lines))
            start_from_end = False
        
        # Generate waypoints with optimized ordering
        for i, line_idx in enumerate(line_order):
            line = all_lines[line_idx]
            start_point = line['start_point']
            end_point = line['end_point']
            
            # Alternate direction for lawnmower pattern, but respect starting direction
            if i == 0:
                # First line uses determined optimal direction
                if start_from_end:
                    waypoints.extend([end_point, start_point])
                else:
                    waypoints.extend([start_point, end_point])
            else:
                # Alternate direction from previous line
                prev_end = waypoints[-1]
                
                # Choose direction that starts closest to where we ended
                start_dist = self.calculate_distance(prev_end, start_point)
                end_dist = self.calculate_distance(prev_end, end_point)
                
                if start_dist < end_dist:
                    waypoints.extend([start_point, end_point])
                else:
                    waypoints.extend([end_point, start_point])
        
        # Final optimization: check if we should reverse entire pattern
        if self.home_position and len(waypoints) > 0:
            first_to_home = self.calculate_distance(waypoints[0], self.home_position)
            last_to_home = self.calculate_distance(waypoints[-1], self.home_position)
            
            print(f"Distance from pattern start to home: {first_to_home:.1f}m")
            print(f"Distance from pattern end to home: {last_to_home:.1f}m")
            
            # If ending closer to home would save significant distance, reverse the pattern
            if last_to_home < first_to_home * 0.8:  # 20% improvement threshold
                print("Reversing entire pattern for better home return")
                waypoints = list(reversed(waypoints))
        
        return waypoints

    def point_at_bearing_and_distance(self, lat: float, lon: float, bearing: float, distance: float) -> Tuple[float, float]:
        """
        Calculate a point at given bearing and distance from a starting point
        """
        R = 6371000  # Earth's radius in meters
        
        lat1 = math.radians(lat)
        lon1 = math.radians(lon)
        bearing_rad = math.radians(bearing)
        
        lat2 = math.asin(math.sin(lat1) * math.cos(distance / R) +
                        math.cos(lat1) * math.sin(distance / R) * math.cos(bearing_rad))
        
        lon2 = lon1 + math.atan2(math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat1),
                                math.cos(distance / R) - math.sin(lat1) * math.sin(lat2))
        
        return math.degrees(lat2), math.degrees(lon2)

    def get_line_intersections_only(self, point_lat: float, point_lon: float, 
                                   bearing: float, polygon: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Get intersection points of a line through a point with given bearing, clipped to polygon boundaries
        """
        # Extend line in both directions from the point
        max_extension = 2000  # 2km in meters
        
        # Calculate end points of the line
        end1_lat, end1_lon = self.point_at_bearing_and_distance(
            point_lat, point_lon, bearing, max_extension
        )
        end2_lat, end2_lon = self.point_at_bearing_and_distance(
            point_lat, point_lon, (bearing + 180) % 360, max_extension
        )
        
        # Find intersections with polygon
        intersections = self.line_polygon_intersections(
            (end1_lat, end1_lon), (end2_lat, end2_lon), polygon
        )
        
        if len(intersections) >= 2:
            # Sort intersections by distance from end1
            intersections.sort(key=lambda p: self.calculate_distance((end1_lat, end1_lon), p))
            return intersections
        
        return []

    def line_polygon_intersections(self, line_start: Tuple[float, float], line_end: Tuple[float, float], 
                                 polygon: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Find intersection points between a line and polygon edges
        """
        intersections = []
        
        for i in range(len(polygon)):
            edge_start = polygon[i]
            edge_end = polygon[(i + 1) % len(polygon)]
            
            intersection = self.line_intersection(line_start, line_end, edge_start, edge_end)
            if intersection:
                intersections.append(intersection)
        
        # Remove duplicates and return
        unique_intersections = []
        for point in intersections:
            is_duplicate = False
            for existing in unique_intersections:
                if abs(point[0] - existing[0]) < 1e-8 and abs(point[1] - existing[1]) < 1e-8:
                    is_duplicate = True
                    break
            if not is_duplicate:
                unique_intersections.append(point)
        
        return unique_intersections

    def line_intersection(self, p1: Tuple[float, float], p2: Tuple[float, float], 
                         p3: Tuple[float, float], p4: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """
        Find intersection point between two line segments
        Returns None if no intersection exists
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4
        
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-10:
            return None  # Lines are parallel
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
        
        if 0 <= u <= 1:  # Intersection is on the polygon edge
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            return (x, y)
        
        return None

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

    def get_polygon_bounds(self, polygon: List[Tuple[float, float]]) -> Tuple[float, float, float, float]:
        """Get bounding box of polygon"""
        lats = [p[0] for p in polygon]
        lons = [p[1] for p in polygon]
        return min(lats), max(lats), min(lons), max(lons)

    def create_waypoint_file(self, waypoints: List[Tuple[float, float]], output_file: str,
                           home_point: Optional[Tuple[float, float]] = None):
        """
        Create enhanced ArduPilot waypoint file
        """
        if not waypoints:
            print("No waypoints to write")
            return

        # Use provided home point or home position or first waypoint
        if home_point is None:
            home_point = self.home_position if self.home_position else waypoints[0]

        try:
            # Ensure output directory exists
            os.makedirs(os.path.dirname(output_file), exist_ok=True)
            
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

                # Survey waypoints
                for lat, lon in waypoints:
                    f.write(f"{waypoint_index}\t0\t3\t16\t0.000000\t0.000000\t0.000000\t0.000000\t"
                           f"{lat:.6f}\t{lon:.6f}\t{self.altitude}.000000\t1\n")
                    waypoint_index += 1

                # RTL command
                f.write(f"{waypoint_index}\t0\t0\t20\t0.000000\t0.000000\t0.000000\t0.000000\t"
                       f"0.000000\t0.000000\t0.000000\t1\n")

            print(f"Enhanced waypoint file created: {output_file}")
            print(f"Total mission items: {waypoint_index + 1}")
            self.print_mission_summary(waypoints, home_point)

        except Exception as e:
            print(f"Error writing waypoint file: {e}")

    def print_mission_summary(self, waypoints: List[Tuple[float, float]], home_point: Tuple[float, float]):
        """Print detailed mission summary"""
        if not waypoints:
            return

        print("\n=== MISSION SUMMARY ===")
        print(f"Home position: {home_point[0]:.6f}, {home_point[1]:.6f}")
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

        # Calculate distances
        total_distance = 0
        for i in range(len(waypoints) - 1):
            lat1, lon1 = waypoints[i]
            lat2, lon2 = waypoints[i + 1]
            total_distance += self.calculate_distance((lat1, lon1), (lat2, lon2))

        # Distance from home to first waypoint
        home_to_first = self.calculate_distance(home_point, waypoints[0]) if waypoints else 0
        
        # Distance from last waypoint to home
        last_to_home = self.calculate_distance(waypoints[-1], home_point) if waypoints else 0

        print(f"Distance from home to first waypoint: {home_to_first:.0f}m")
        print(f"Estimated survey flight distance: {total_distance:.0f}m")
        print(f"Distance from last waypoint to home: {last_to_home:.0f}m")
        print(f"Total mission distance: {home_to_first + total_distance + last_to_home:.0f}m")
        print(f"Pattern type: Corner-optimized parallel flight with smart ordering")
        print("========================\n")

def main():
    """Enhanced main function with corner-optimized waypoint ordering"""
    if len(sys.argv) < 3:
        print("KML to ArduPilot Waypoint Converter v8")
        print("Usage: python kml_to_wayp_v8.py <input.kml> <altitude> [options]")
        print("\nRequired arguments:")
        print("  input.kml     Path to KML polygon file")
        print("  altitude      Flight altitude in meters (integer)")
        print("\nOptional arguments:")
        print("  --spacing METERS        Line spacing in meters (default: auto-calculated)")
        print("  --fence-padding METERS  Distance to stay inside fence boundaries (default: 2m)")
        print("  --home-lat LATITUDE     Home position latitude (for optimization)")
        print("  --home-lon LONGITUDE    Home position longitude (for optimization)")
        print("  --no-camera            Disable camera triggers")
        print("  --trigger-dist METERS   Distance between photos (default: 5m)")
        print("  --gimbal-tilt DEGREES   Camera tilt angle (default: -90°)")
        print("  --overlap PERCENT       Photo overlap percentage (default: 80%)")
        print("  --sidelap PERCENT       Side overlap percentage (default: 60%)")
        print("\nNEW in v8:")
        print("  Smart corner detection: Identifies polygon corners for optimal flight planning")
        print("  Optimized start/end: Starts from top-right area, ends near top-left")
        print("  Minimal return distance: Reduces total flight time by optimizing pattern direction")
        print("  Enhanced ordering: Considers home position for best waypoint sequence")
        print("\nExample:")
        print("  python kml_to_wayp_v8.py survey_area.kml 50 --spacing 8 --fence-padding 5 --home-lat 12.345678 --home-lon 78.901234")
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
    fence_padding = 2.0  # Default 2m fence padding
    camera_enabled = True
    trigger_distance = 5
    gimbal_tilt = -90
    overlap_percent = 80
    sidelap_percent = 60
    home_lat = None
    home_lon = None

    i = 3
    while i < len(sys.argv):
        if sys.argv[i] == '--spacing' and i + 1 < len(sys.argv):
            spacing = float(sys.argv[i + 1]) / 111000  # Convert meters to degrees
            i += 2
        elif sys.argv[i] == '--fence-padding' and i + 1 < len(sys.argv):
            fence_padding = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--buffer' and i + 1 < len(sys.argv):
            # Keep backward compatibility with old --buffer option
            fence_padding = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--home-lat' and i + 1 < len(sys.argv):
            home_lat = float(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--home-lon' and i + 1 < len(sys.argv):
            home_lon = float(sys.argv[i + 1])
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
            print(f"Unknown argument: {sys.argv[i]}")
            i += 1

    # Set default spacing if not provided
    if spacing is None:
        spacing = 0.00005  # Default ~5m spacing

    # Convert fence padding to degrees
    buffer_distance = fence_padding / 111000

    # Set home position if provided
    home_position = None
    if home_lat is not None and home_lon is not None:
        home_position = (home_lat, home_lon)
        print(f"Home position set to: {home_lat:.6f}, {home_lon:.6f}")
    else:
        print("No home position provided - optimization will be limited")

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
    output_file = os.path.expanduser("~/ardu-sim/waypoints_v8.txt")
    
    # Create converter
    converter = KMLToWaypointV8(
        altitude=altitude,
        spacing=spacing,
        buffer_distance=buffer_distance,
        camera_settings=camera_settings,
        home_position=home_position
    )

    # Parse KML file
    print(f"Parsing KML file: {input_file}")
    polygon = converter.parse_kml(input_file)

    if not polygon:
        print("Failed to parse KML file or no valid polygon found")
        return

    print(f"Fence padding: {fence_padding}m")
    print("Using corner-optimized parallel flight pattern...")
    
    # Generate waypoints using optimized pattern
    waypoints = converter.generate_parallel_lines_to_longest_side(polygon)

    if not waypoints:
        print("No waypoints generated within polygon boundary")
        return

    # Create waypoint file
    converter.create_waypoint_file(waypoints, output_file)

    print(f"Conversion complete! Output saved as: {output_file}")

if __name__ == "__main__":
    main()
