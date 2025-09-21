import serial
import sys
import os

# Add project root to Python path for direct execution
if __name__ == "__main__":
    project_root = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', '..'))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

# Smart import strategy - try src. prefix first, then without
try:
    import src.log as log
except ImportError:
    # Running from src directory, use relative imports
    import log

logger = log.getLogger(__name__)

port = "/dev/ttyAMA0"  # update if different (not /dev/serial0 :) )
baudrate = 9600        # or 38400 / 115200 if you changed it


def parse_nmea_coordinate(coord_str, direction):
    """
    Parse NMEA coordinate format (DDMM.MMMM) to decimal degrees.
    
    Args:
        coord_str (str): Coordinate in NMEA format (e.g., "4916.45")
        direction (str): Direction ('N', 'S', 'E', 'W')
    
    Returns:
        float: Coordinate in decimal degrees
    """
    if not coord_str or not direction:
        return None
    
    try:
        # NMEA format: DDMM.MMMM (degrees + minutes)
        if len(coord_str) < 4:
            return None
        
        # Find decimal point
        decimal_idx = coord_str.find('.')
        if decimal_idx == -1:
            return None
        
        # Extract degrees and minutes
        if decimal_idx >= 4:  # Longitude format DDDMM.MMMM
            degrees = int(coord_str[:decimal_idx-2])
            minutes = float(coord_str[decimal_idx-2:])
        else:  # Latitude format DDMM.MMMM
            degrees = int(coord_str[:decimal_idx-2])
            minutes = float(coord_str[decimal_idx-2:])
        
        # Convert to decimal degrees
        decimal_degrees = degrees + (minutes / 60.0)
        
        # Apply direction
        if direction in ['S', 'W']:
            decimal_degrees = -decimal_degrees
        
        return decimal_degrees
    
    except (ValueError, IndexError) as e:
        logger.error(f"Error parsing coordinate {coord_str} {direction}: {e}")
        return None


def parse_gga_sentence(sentence):
    """
    Parse GGA (Global Positioning System Fix Data) sentence.
    
    Format: $GPGGA,time,lat,lat_dir,lon,lon_dir,quality,satellites,hdop,altitude,alt_unit,geoid,geoid_unit,dgps_time,dgps_id*checksum
    
    Args:
        sentence (str): NMEA GGA sentence
    
    Returns:
        dict: Parsed GPS data or None if parsing fails
    """
    try:
        parts = sentence.split(',')
        if len(parts) < 15:
            return None
        
        # Extract relevant fields
        time_utc = parts[1]
        lat_str = parts[2]
        lat_dir = parts[3]
        lon_str = parts[4]
        lon_dir = parts[5]
        quality = parts[6]
        satellites = parts[7]
        hdop = parts[8]
        altitude = parts[9]
        
        # Parse coordinates
        latitude = parse_nmea_coordinate(lat_str, lat_dir)
        longitude = parse_nmea_coordinate(lon_str, lon_dir)
        
        if latitude is None or longitude is None:
            return None
        
        return {
            'sentence_type': 'GGA',
            'time_utc': time_utc,
            'latitude': latitude,
            'longitude': longitude,
            'quality': int(quality) if quality else 0,
            'satellites': int(satellites) if satellites else 0,
            'hdop': float(hdop) if hdop else 0.0,
            'altitude': float(altitude) if altitude else 0.0
        }
    
    except (ValueError, IndexError) as e:
        logger.error(f"Error parsing GGA sentence: {e}")
        return None


def parse_rmc_sentence(sentence):
    """
    Parse RMC (Recommended Minimum Course) sentence.
    
    Format: $GPRMC,time,status,lat,lat_dir,lon,lon_dir,speed,course,date,mag_var,mag_var_dir*checksum
    
    Args:
        sentence (str): NMEA RMC sentence
    
    Returns:
        dict: Parsed GPS data or None if parsing fails
    """
    try:
        parts = sentence.split(',')
        if len(parts) < 12:
            return None
        
        # Extract relevant fields
        time_utc = parts[1]
        status = parts[2]
        lat_str = parts[3]
        lat_dir = parts[4]
        lon_str = parts[5]
        lon_dir = parts[6]
        speed = parts[7]
        course = parts[8]
        date = parts[9]
        
        # Only process if status is 'A' (Active/Valid)
        if status != 'A':
            return None
        
        # Parse coordinates
        latitude = parse_nmea_coordinate(lat_str, lat_dir)
        longitude = parse_nmea_coordinate(lon_str, lon_dir)
        
        if latitude is None or longitude is None:
            return None
        
        return {
            'sentence_type': 'RMC',
            'time_utc': time_utc,
            'latitude': latitude,
            'longitude': longitude,
            'status': status,
            'speed_knots': float(speed) if speed else 0.0,
            'course': float(course) if course else 0.0,
            'date': date
        }
    
    except (ValueError, IndexError) as e:
        logger.error(f"Error parsing RMC sentence: {e}")
        return None


def parse_nmea_sentence(line):
    """
    Parse a single NMEA sentence.
    
    Args:
        line (str): NMEA sentence line
    
    Returns:
        dict: Parsed GPS data or None if parsing fails
    """
    line = line.strip()
    
    # Check for valid NMEA sentence start
    if not line.startswith('$'):
        return None
    
    # Remove checksum if present
    if '*' in line:
        line = line.split('*')[0]
    
    # Determine sentence type and parse accordingly
    if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
        return parse_gga_sentence(line)
    elif line.startswith('$GPRMC') or line.startswith('$GNRMC'):
        return parse_rmc_sentence(line)
    
    return None


def get_gnss_data(timeout_attempts=10):
    """
    Get GNSS data from the GPS module.
    
    Args:
        timeout_attempts (int): Number of attempts to read valid data
    
    Returns:
        tuple: (latitude, longitude) or (None, None) if no valid data
    """
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            logger.debug(f"Connected to GNSS on {port} at {baudrate} baud")
            
            for attempt in range(timeout_attempts):
                try:
                    line = ser.readline().decode('ascii', errors='replace').strip()
                    if not line:
                        continue
                    
                    logger.debug(f"GNSS raw: {line}")
                    
                    parsed_data = parse_nmea_sentence(line)
                    if parsed_data and 'latitude' in parsed_data and 'longitude' in parsed_data:
                        logger.info(f"GNSS fix: {parsed_data['latitude']:.6f}, {parsed_data['longitude']:.6f}")
                        return parsed_data['latitude'], parsed_data['longitude']
                
                except serial.SerialException as e:
                    logger.error(f"Serial error reading GNSS: {e}")
                    break
                except Exception as e:
                    logger.error(f"Error processing GNSS data: {e}")
                    continue
            
            logger.warning(f"No valid GNSS data after {timeout_attempts} attempts")
            return None, None
    
    except serial.SerialException as e:
        logger.error(f"Failed to connect to GNSS on {port}: {e}")
        return None, None
    except Exception as e:
        logger.error(f"Unexpected error in get_gnss_data: {e}")
        return None, None


def get_detailed_gnss_data(timeout_attempts=10):
    """
    Get detailed GNSS data from the GPS module.
    
    Args:
        timeout_attempts (int): Number of attempts to read valid data
    
    Returns:
        dict: Detailed GPS data or None if no valid data
    """
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            logger.debug(f"Connected to GNSS on {port} at {baudrate} baud")
            
            for attempt in range(timeout_attempts):
                try:
                    line = ser.readline().decode('ascii', errors='replace').strip()
                    if not line:
                        continue
                    
                    logger.debug(f"GNSS raw: {line}")
                    
                    parsed_data = parse_nmea_sentence(line)
                    if parsed_data and 'latitude' in parsed_data and 'longitude' in parsed_data:
                        logger.info(f"GNSS detailed data: {parsed_data}")
                        return parsed_data
                
                except serial.SerialException as e:
                    logger.error(f"Serial error reading GNSS: {e}")
                    break
                except Exception as e:
                    logger.error(f"Error processing GNSS data: {e}")
                    continue
            
            logger.warning(f"No valid GNSS data after {timeout_attempts} attempts")
            return None
    
    except serial.SerialException as e:
        logger.error(f"Failed to connect to GNSS on {port}: {e}")
        return None
    except Exception as e:
        logger.error(f"Unexpected error in get_detailed_gnss_data: {e}")
        return None


if __name__ == "__main__":
    logger.info(f"Starting GNSS reader on {port} at {baudrate} baud")
    
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            logger.info("Connected to GNSS module. Reading data...")
            
            while True:
                try:
                    line = ser.readline().decode('ascii', errors='replace').strip()
                    if not line:
                        continue
                    
                    parsed_data = parse_nmea_sentence(line)
                    if parsed_data and 'latitude' in parsed_data and 'longitude' in parsed_data:
                        print(f"{parsed_data['latitude']:.6f},{parsed_data['longitude']:.6f}")
                        if 'sentence_type' in parsed_data:
                            print(f"  Type: {parsed_data['sentence_type']}")
                        if 'altitude' in parsed_data:
                            print(f"  Altitude: {parsed_data['altitude']:.1f}m")
                        if 'satellites' in parsed_data:
                            print(f"  Satellites: {parsed_data['satellites']}")
                        print()
                
                except KeyboardInterrupt:
                    logger.info("Interrupted by user")
                    break
                except serial.SerialException as e:
                    logger.error(f"Serial error: {e}")
                    break
                except Exception as e:
                    logger.error(f"Error processing data: {e}")
                    continue
    
    except serial.SerialException as e:
        logger.error(f"Failed to connect to GNSS on {port}: {e}")
        print(f"Error: Could not connect to GNSS module on {port}")
        print("Make sure the GPS module is connected and the port is correct.")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        print(f"Unexpected error: {e}")