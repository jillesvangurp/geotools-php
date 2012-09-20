<?php
/**
 * Copyright (c) 2012, Jilles van Gurp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

class GeoGeometry {
	/**
	 * Earth's mean radius, in meters.
	 *
	 * @see http://en.wikipedia.org/wiki/Earth%27s_radius#Mean_radii
	 */
	protected $EARTH_RADIUS = 6371000.0;

	protected $EARTH_RADIUS_METERS = 6371000.0;
	protected $EARTH_CIRCUMFERENCE_METERS;
	protected $DEGREE_LATITUDE_METERS;

	function __construct() {
		$this -> EARTH_CIRCUMFERENCE_METERS = 6371000.0 * pi() * 2.0;
		$this -> DEGREE_LATITUDE_METERS = 6371000.0 * pi() / 180.0;
	}

	/**
	 * @param points
	 *            points that make up the polygon as array of arrays of
	 *            [latitude,longitude]
	 * @return bounding box that contains the polygon as a double array of
	 *         [minLat,maxLat,minLon,maxLon}
	 */
	function polygonToBbox($points) {
		$minLat = 91;
		$minLon = 181;
		$maxLat = -91;
		$maxLon = -181;

		for ($i = 0; $i < sizeof($points); $i++) {
			$minLat = min($minLat, $points[$i][0]);
			$minLon = min($minLon, $points[$i][1]);
			$maxLat = max($maxLat, $points[$i][0]);
			$maxLon = max($maxLon, $points[$i][1]);
		}

		return array($minLat, $maxLat, $minLon, $maxLon);
	}

	/**
	 * @param bbox
	 *            double array of [minLat,maxLat,minLon,maxLon}
	 * @param latitude
	 * @param longitude
	 * @return true if the latitude and longitude are contained in the bbox
	 */
	function bboxContains($bbox, $latitude, $longitude) {
		return $bbox[0] <= $latitude && $latitude <= $bbox[1] && $bbox[2] <= $longitude && $longitude <= $bbox[3];
	}

	/**
	 * Determine whether a point is contained in a polygon. Note, technically
	 * the points that make up the polygon are not contained by it.
	 *
	 * @param polygonPoints
	 *            polygonPoints points that make up the polygon as array of arrays of
	 *            [latitude,longitude]
	 * @param latitude
	 * @param longitude
	 * @return true if the polygon contains the coordinate
	 */
	function polygonContains($polygonPoints, $latitude, $longitude) {

		if (sizeof($polygonPoints) < 3) {
			throw new InvalidArgumentException("a polygon must have at least three points");
		}

		$bbox = $this -> polygonToBbox($polygonPoints);
		if (!$this -> bboxContains($bbox, $latitude, $longitude)) {
			// outside the containing bbox
			return FALSE;
		}

		$hits = 0;

		$lastLatitude = $polygonPoints[sizeof($polygonPoints) - 1][0];
		$lastLongitude = $polygonPoints[sizeof($polygonPoints) - 1][1];

		// Walk the edges of the polygon
		for ($i = 0; $i < sizeof($polygonPoints); $lastLatitude = $currentLatitude, $lastLongitude = $currentLongitude, $i++) {
			$currentLatitude = $polygonPoints[$i][0];
			$currentLongitude = $polygonPoints[$i][1];

			if ($currentLongitude == $lastLongitude) {
				continue;
			}

			$leftLatitude;
			if ($currentLatitude < $lastLatitude) {
				if ($latitude >= $lastLatitude) {
					continue;
				}
				$leftLatitude = $currentLatitude;
			} else {
				if ($latitude >= $currentLatitude) {
					continue;
				}
				$leftLatitude = $lastLatitude;
			}

			if ($currentLongitude < $lastLongitude) {
				if ($longitude < $currentLongitude || $longitude >= $lastLongitude) {
					continue;
				}
				if ($latitude < $leftLatitude) {
					$hits++;
					continue;
				}
				$test1 = $latitude - $currentLatitude;
				$test2 = $longitude - $currentLongitude;
			} else {
				if ($longitude < $lastLongitude || $longitude >= $currentLongitude) {
					continue;
				}
				if ($latitude < $leftLatitude) {
					$hits++;
					continue;
				}
				$test1 = $latitude - $lastLatitude;
				$test2 = $longitude - $lastLongitude;
			}

			if ($test1 < $test2 / ($lastLongitude - $currentLongitude) * ($lastLatitude - $currentLatitude)) {
				$hits++;
			}
		}

		return ($hits & 1) != 0;
	}

	/**
	 * Simple rounding method that allows you to get rid of some decimals in a
	 * double.
	 *
	 * @param d
	 * @param decimals maximum is 17
	 * @return d rounded to the specified precision
	 */
	function roundToDecimals($d, $decimals) {
		if ($decimals > 17) {
			throw new InvalidArgumentException("are you sure you want this many decimals?");
		}
		$factor = pow(10, $decimals);
		return round($d * $factor) / $factor;
	}

	/**
	 * @param d degrees
	 * @return the radian for the decimal degree:  $d / 180 * pi().
	 */
	function toRadians($d) {
		return $d / 180 * pi();
	}

	private function lengthOfLongitudeDegreeAtLatitude($latitude) {
		$latitudeInRadians = $this -> toRadians($latitude);
		return cos($latitudeInRadians) * $this -> EARTH_CIRCUMFERENCE_METERS / 360.0;
	}

	/**
	 * Translate a point along the longitude for the specified amount of meters.
	 * Note, this method assumes the earth is a sphere and the result is not
	 * going to be very precise for larger distances.
	 *
	 * @param latitude
	 * @param longitude
	 * @param meters
	 * @return the translated coordinate.
	 */
	function translateLongitude($latitude, $longitude, $meters) {
		return array($latitude, $longitude + $meters / $this -> lengthOfLongitudeDegreeAtLatitude($latitude));
	}

	/**
	 * Translate a point along the latitude for the specified amount of meters.
	 * Note, this method assumes the earth is a sphere and the result is not
	 * going to be very precise for larger distances.
	 *
	 * @param latitude
	 * @param longitude
	 * @param meters
	 * @return the translated coordinate.
	 */
	function translateLatitude($latitude, $longitude, $meters) {
		return array($latitude + $meters / $this -> DEGREE_LATITUDE_METERS, $longitude);
	}

	/**
	 * Translate a point by the specified meters along the longitude and
	 * latitude. Note, this method assumes the earth is a sphere and the result
	 * is not going to be very precise for larger distances.
	 *
	 * @param latitude
	 * @param longitude
	 * @param lateralMeters
	 * @param longitudalMeters
	 * @return the translated coordinate.
	 */
	function translate($latitude, $longitude, $lateralMeters, $longitudalMeters) {
		$longitudal = $this -> translateLongitude($latitude, $longitude, $longitudalMeters);
		return $this -> translateLatitude($longitudal[0], $longitudal[1], $lateralMeters);
	}

	/**
	 * Compute the Haversine distance between the two coordinates. Haversine is
	 * one of several distance calculation algorithms that exist. It is not very
	 * precise in the sense that it assumes the earth is a perfect sphere, which
	 * it is not. This means precision drops over larger distances. According to
	 * http://en.wikipedia.org/wiki/Haversine_formula there is a 0.5% error
	 * margin given the 1% difference in curvature between the equator and the
	 * poles.
	 *
	 * @param firstCoordinate
	 *            [latitude, longitude]
	 * @param secondCoordinate
	 *            [latitude, longitude]
	 * @return the distance in meters
	 *
	 */
	function distance($point1, $point2) {
		$lat1 = $point1[0];
		$long1 = $point1[1];

		$lat2 = $point2[0];
		$long2 = $point2[1];

		$deltaLat = $this -> toRadians($lat2 - $lat1);
		$deltaLon = $this -> toRadians($long2 - $long1);

		$a = sin($deltaLat / 2) * sin($deltaLat / 2) + cos($this -> toRadians($lat1)) * cos($this -> toRadians($lat2)) * sin($deltaLon / 2) * sin($deltaLon / 2);

		$c = 2 * asin(sqrt($a));

		return $this -> EARTH_RADIUS * $c;
	}

	function linesCross($x1, $y1, $x2, $y2, $u1, $v1, $u2, $v2) {
		// formula for line: y= a+bx

		// vertical lines result in a divide by 0;
		$line1Vertical = $x2 == $x1;
		$line2Vertical = $u2 == $u1;
		if ($line1Vertical && $line2Vertical) {
			// x=a
			if ($x1 == $u1) {
				// lines are the same
				return $y1 <= $v1 && $v1 < $y2 || $y1 <= $v2 && $v2 < $y2;
			} else {
				// parallel -> they don't intersect!
				return FALSE;
			}
		} else if ($line1Vertical && !$line2Vertical) {
			$b2 = ($v2 - $v1) / ($u2 - $u1);
			$a2 = $v1 - $b2 * $u1;

			$xi = $x1;
			$yi = $a2 + $b2 * $xi;

			return $yi >= $y1 && $yi <= $y2;

		} else if (!$line1Vertical && $line2Vertical) {
			$b1 = ($y2 - $y1) / ($x2 - $x1);
			$a1 = $y1 - $b1 * $x1;

			$xi = $u1;
			$yi = $a1 + $b1 * $xi;

			return $yi >= $v1 && $yi <= $v2;
		} else {

			$b1 = ($y2 - $y1) / ($x2 - $x1);
			// divide by zero if second line vertical
			$b2 = ($v2 - $v1) / ($u2 - $u1);

			$a1 = $y1 - $b1 * $x1;
			$a2 = $v1 - $b2 * $u1;

			if ($b1 - $b2 == 0) {
				if ($a1 == $a2) {
					// lines are the same
					return $x1 <= $u1 && $u1 < $x2 || $x1 <= $u2 && $u2 < $x2;
				} else {
					// parallel -> they don't intersect!
					return FALSE;
				}
			}
			// calculate intersection point xi,yi
			$xi = -($a1 - $a2) / ($b1 - $b2);
			$yi = $a1 + $b1 * $xi;
			if (($x1 - $xi) * ($xi - $x2) >= 0 && ($u1 - $xi) * ($xi - $u2) >= 0 && ($y1 - $yi) * ($yi - $y2) >= 0 && ($v1 - $yi) * ($yi - $v2) >= 0) {
				return TRUE;
			} else {
				return FALSE;
			}
		}
	}

	/**
	 * Converts a circle to a polygon.
	 *
	 * @param segments
	 *            number of segments the polygon should have. The higher this
	 *            number, the better of an approximation the polygon is for the
	 *            circle.
	 * @param latitude
	 * @param longitude
	 * @param radius
	 * @return an array of the points [latitude,longitude] that make up the
	 *         polygon.
	 */
	function circle2polygon($segments, $latitude, $longitude, $radius) {
		if ($segments < 5) {
			throw new InvalidArgumentException("you need a minimum of 5 segments");
		}
		// for n segments you need n+1 points
		$points = array();

		$relativeLatitude = $radius / $this -> EARTH_RADIUS_METERS * 180 / pi();
		$relativeLongitude = $relativeLatitude / cos($this -> toRadians($latitude));

		for ($i = 0; $i < $segments + 1; $i++) {
			// radians go from 0 to 2*PI; we want to divide the circle in nice
			// segments
			$theta = 2 * pi() * $i / $segments;

			// on the unit circle, any point of the circle has the coordinate
			// cos(t),sin(t) where t is the radian. So, all we need to do that
			// is multiply that with the relative latitude and longitude
			// note, latitude takes the role of y, not x. By convention we
			// always note latitude, longitude instead of the other way around
			$latOnCircle = $latitude + $relativeLatitude * sin($theta);
			$lonOnCircle = $longitude + $relativeLongitude * cos($theta);
			if ($lonOnCircle > 180) {
				$lonOnCircle = -180 + ($lonOnCircle - 180);
			} else if ($lonOnCircle < -180) {
				$lonOnCircle = 180 - ($lonOnCircle + 180);
			}

			array_push($points, array($latOnCircle, $lonOnCircle));
		}
		return $points;
	}

}

class GeoHash {
	protected $BITS = array(16, 8, 4, 2, 1);
	protected $BASE32_CHARS = array('0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'j', 'k', 'm', 'n', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z');
	protected $BASE32_DECODE_MAP = array();
	protected $geogeometry;

	function __construct() {
		$this -> geogeometry = new GeoGeometry();
		for ($i = 0; $i < count($this -> BASE32_CHARS); $i++) {
			$this -> BASE32_DECODE_MAP[$this -> BASE32_CHARS[$i]] = $i;
		}
	}

	/**
	 * Encodes a coordinate into a geo hash.
	 *
	 * @see "http://en.wikipedia.org/wiki/Geohash"
	 * @param latitude
	 * @param longitude
	 * @param length, optional defaults to 12 (maximum). The minimum is 1.
	 * @return geo hash for the coordinate
	 */
	function encode($latitude, $longitude, $length = 12) {
		if ($length < 1 || $length > 12) {
			throw new InvalidArgumentException("length should be between 1 and 12");
		}
		$latInterval = array(-90.0, 90.0);
		$lonInterval = array(-180.0, 180.0);

		$geoHash = '';
		$isEven = TRUE;
		$bit = 0;
		$ch = 0;

		while (strlen($geoHash) < $length) {
			$mid = 0.0;
			if ($isEven) {
				$mid = ($lonInterval[0] + $lonInterval[1]) / 2;
				if ($longitude > $mid) {
					$ch |= $this -> BITS[$bit];
					$lonInterval[0] = $mid;
				} else {
					$lonInterval[1] = $mid;
				}

			} else {
				$mid = ($latInterval[0] + $latInterval[1]) / 2;
				if ($latitude > $mid) {
					$ch |= $this -> BITS[$bit];
					$latInterval[0] = $mid;
				} else {
					$latInterval[1] = $mid;
				}
			}

			$isEven = $isEven ? FALSE : TRUE;

			if ($bit < 4) {
				$bit++;
			} else {
				$geoHash = $geoHash . $this -> BASE32_CHARS[$ch];
				$bit = 0;
				$ch = 0;
			}
		}
		return $geoHash;
	}

	private function encodeWithBbox($latitude, $longitude, $length = 12) {
		if ($length < 1 || $length > 12) {
			throw new InvalidArgumentException("length should be between 1 and 12");
		}
		$latInterval = array(-90.0, 90.0);
		$lonInterval = array(-180.0, 180.0);

		$geoHash = '';
		$isEven = TRUE;
		$bit = 0;
		$ch = 0;

		while (strlen($geoHash) < $length) {
			$mid = 0.0;
			if ($isEven) {
				$mid = ($lonInterval[0] + $lonInterval[1]) / 2;
				if ($longitude > $mid) {
					$ch |= $this -> BITS[$bit];
					$lonInterval[0] = $mid;
				} else {
					$lonInterval[1] = $mid;
				}

			} else {
				$mid = ($latInterval[0] + $latInterval[1]) / 2;
				if ($latitude > $mid) {
					$ch |= $this -> BITS[$bit];
					$latInterval[0] = $mid;
				} else {
					$latInterval[1] = $mid;
				}
			}

			$isEven = $isEven ? FALSE : TRUE;

			if ($bit < 4) {
				$bit++;
			} else {
				$geoHash = $geoHash . $this -> BASE32_CHARS[$ch];
				$bit = 0;
				$ch = 0;
			}
		}
		return array($geoHash, array($latInterval[0], $latInterval[1], $lonInterval[0], $lonInterval[1]));
	}

	/**
	 * This decodes the geo hash to it's center. Note that the coordinate that
	 * you used to generate the geo hash may be anywhere in the geo hash's
	 * bounding box and therefore you should not expect them to be identical.
	 *
	 * Should you wish to apply some rounding, you can use the
	 * GeoGeometry.roundToDecimals method.
	 *
	 * @param geohash
	 * @return a coordinate representing the center of the geohash as a double
	 *         array of [latitude,longitude]
	 */
	function decode($geoHash) {
		$latInterval = array(-90.0, 90.0);
		$lonInterval = array(-180.0, 180.0);
		$isEven = TRUE;

		for ($i = 0; $i < strlen($geoHash); $i++) {
			$currentChar = $this -> BASE32_DECODE_MAP[$geoHash[$i]];
			for ($z = 0; $z < sizeof($this -> BITS); $z++) {
				$mask = $this -> BITS[$z];
				if ($isEven) {
					if (($currentChar & $mask) != 0) {
						$lonInterval[0] = ($lonInterval[0] + $lonInterval[1]) / 2;
					} else {
						$lonInterval[1] = ($lonInterval[0] + $lonInterval[1]) / 2;
					}

				} else {
					if (($currentChar & $mask) != 0) {
						$latInterval[0] = ($latInterval[0] + $latInterval[1]) / 2;
					} else {
						$latInterval[1] = ($latInterval[0] + $latInterval[1]) / 2;
					}
				}
				$isEven = $isEven ? FALSE : TRUE;
			}
		}
		$latitude = ($latInterval[0] + $latInterval[1]) / 2;
		$longitude = ($lonInterval[0] + $lonInterval[1]) / 2;

		return array($latitude, $longitude);
	}

	/**
	 * @param geohash
	 * @return double array representing the bounding box for the geohash of
	 *         [nort latitude, south latitude, east longitude, west longitude]
	 */
	function decodeToBbox($geoHash) {
		$latInterval = array(-90.0, 90.0);
		$lonInterval = array(-180.0, 180.0);
		$isEven = TRUE;

		for ($i = 0; $i < strlen($geoHash); $i++) {
			$currentChar = $this -> BASE32_DECODE_MAP[$geoHash[$i]];
			for ($z = 0; $z < sizeof($this -> BITS); $z++) {
				$mask = $this -> BITS[$z];
				if ($isEven) {
					if (($currentChar & $mask) != 0) {
						$lonInterval[0] = ($lonInterval[0] + $lonInterval[1]) / 2;
					} else {
						$lonInterval[1] = ($lonInterval[0] + $lonInterval[1]) / 2;
					}

				} else {
					if (($currentChar & $mask) != 0) {
						$latInterval[0] = ($latInterval[0] + $latInterval[1]) / 2;
					} else {
						$latInterval[1] = ($latInterval[0] + $latInterval[1]) / 2;
					}
				}
				$isEven = $isEven ? FALSE : TRUE;
			}
		}

		return array($latInterval[0], $latInterval[1], $lonInterval[0], $lonInterval[1]);
	}

	/**
	 * @return the geo hash of the same length directly north of the bounding
	 *         box.
	 */
	function north($geoHash) {
		$bbox = $this -> decodeToBbox($geoHash);
		$latDiff = $bbox[1] - $bbox[0];
		$lat = $bbox[0] - $latDiff / 2;
		$lon = ($bbox[2] + $bbox[3]) / 2;
		return $this -> encode($lat, $lon, strlen($geoHash));
	}

	/**
	 * @return the geo hash of the same length directly south of the bounding
	 *         box.
	 */
	function south($geoHash) {
		$bbox = $this -> decodeToBbox($geoHash);
		$latDiff = $bbox[1] - $bbox[0];
		$lat = $bbox[1] + $latDiff / 2;
		$lon = ($bbox[2] + $bbox[3]) / 2;
		return $this -> encode($lat, $lon, strlen($geoHash));
	}

	/**
	 * @return the geo hash of the same length directly west of the bounding
	 *         box.
	 */
	function west($geoHash) {
		$bbox = $this -> decodeToBbox($geoHash);
		$lonDiff = $bbox[3] - $bbox[2];
		$lat = ($bbox[0] + $bbox[1]) / 2;
		$lon = $bbox[2] - $lonDiff / 2;
		if ($lon < -180) {
			$lon = 180 - ($lon + 180);
		}

		return $this -> encode($lat, $lon, strlen($geoHash));
	}

	/**
	 * @return the geo hash of the same length directly east of the bounding
	 *         box.
	 */
	function east($geoHash) {
		$bbox = $this -> decodeToBbox($geoHash);
		$lonDiff = $bbox[3] - $bbox[2];
		$lat = ($bbox[0] + $bbox[1]) / 2;
		$lon = $bbox[3] + $lonDiff / 2;

		if ($lon > 180) {
			$lon = -180 + ($lon - 180);
		}

		return $this -> encode($lat, $lon, strlen($geoHash));
	}

	function isWest($l1, $l2) {
		$ll1 = $l1 + 180;
		$ll2 = $l2 + 180;
		if ($ll1 < $ll2 && $ll2 - $ll1 < 180) {
			return TRUE;
		} else if ($ll1 > $ll2 && $ll2 + 360 - $ll1 < 180) {
			return TRUE;
		} else {
			return FALSE;
		}
	}

	function isEast($l1, $l2) {
		$ll1 = $l1 + 180;
		$ll2 = $l2 + 180;
		if ($ll1 > $ll2 && $ll1 - $ll2 < 180) {
			return TRUE;
		} else if ($ll1 < $ll2 && $ll1 + 360 - $ll2 < 180) {
			return TRUE;
		} else {
			return FALSE;
		}
	}

	function isNorth($l1, $l2) {
		return $l1 > $l2;
	}

	function isSouth($l1, $l2) {
		return $l1 < $l2;
	}

	/**
	 * Returns a suitable geo hash length for the desired granularity in meters. The maximum length returned here is 8.
	 * @param granularityInMeters
	 * @return a length between 2 and 10.
	 */
	function getSuitableHashLength($granularityInMeters) {
		if ($granularityInMeters < 1) {
			$hashLength = 10;
		} else if ($granularityInMeters < 5) {
			$hashLength = 9;
		} else if ($granularityInMeters < 50) {
			$hashLength = 8;
		} else if ($granularityInMeters < 200) {
			$hashLength = 7;
		} else if ($granularityInMeters < 1500) {
			$hashLength = 6;
		} else if ($granularityInMeters < 10000) {
			$hashLength = 5;
		} else if ($granularityInMeters < 50000) {
			$hashLength = 4;
		} else if ($granularityInMeters < 200000) {
			$hashLength = 3;
		} else {
			$hashLength = 2;
		}
		return $hashLength;
	}

	/**
	 * Cover the polygon with geo hashes. This is useful for indexing mainly.
	 *
	 * @param maxLength
	 *            maximum length of the geoHash; the more you specify, the more
	 *            expensive it gets
	 * @param polygonPoints
	 *            polygonPoints points that make up the polygon as arrays of
	 *            [latitude,longitude]
	 * @return a set of geo hashes that cover the polygon area.
	 */
	function getGeoHashesForPolygon($maxLength, $polygonPoints) {
		if ($maxLength < 2 || $maxLength > 10) {
			throw new InvalidArgumentException("maxLength should be between 1 and 10");
		}

		$bbox = $this -> geogeometry -> polygonToBbox($polygonPoints);
		// first lets figure out an appropriate geohash length
		$diagonal = $this -> geogeometry -> distance(array($bbox[0], $bbox[2]), array($bbox[1], $bbox[3]));
		$hashLength = $this -> getSuitableHashLength($diagonal);

		$partiallyContained = array();
		// now lets generate all geohashes for the containing bounding box
		// lets start at the top left:

		$rowHash = $this -> encode($bbox[0], $bbox[2], $hashLength);
		$rowBox = $this -> decodeToBbox($rowHash);
		while ($rowBox[0] < $bbox[1]) {
			$columnHash = $rowHash;
			$columnBox = $rowBox;

			while ($this -> isWest($columnBox[2], $bbox[3])) {
				array_push($partiallyContained, $columnHash);
				$columnHash = $this -> east($columnHash);
				$columnBox = $this -> decodeToBbox($columnHash);
			}

			// move to the next row
			$rowHash = $this -> south($rowHash);
			$rowBox = $this -> decodeToBbox($rowHash);
		}

		$fullyContained = array();

		$detail = $hashLength;
		// we're not aiming for perfect detail here in terms of 'pixelation', 6
		// extra chars in the geohash ought to be enough and going beyond 9
		// doesn't serve much purpose.

		while ($detail < $maxLength) {
			$result = $this -> splitAndFilter($polygonPoints, $fullyContained, $partiallyContained);
			$partiallyContained = $result[0];
			$fullyContained = $result[1];
			$detail = $detail + 1;
		}

		// add the remaining hashes that we didn't split
		foreach ($partiallyContained as $value) {
			array_push($fullyContained, $value);
		}

		return $fullyContained;
	}

	private function splitAndFilter($polygonPoints, $fullyContained, $partiallyContained) {
		$stillPartial = array();
		// now we need to break up the partially contained hashes
		foreach ($partiallyContained as $hash) {
			foreach ($this->subHashes($hash) as $h) {
				$hashBbox = $this -> decodeToBbox($h);
				$nw = $this -> geogeometry -> polygonContains($polygonPoints, $hashBbox[0], $hashBbox[2]);
				$ne = $this -> geogeometry -> polygonContains($polygonPoints, $hashBbox[0], $hashBbox[3]);
				$sw = $this -> geogeometry -> polygonContains($polygonPoints, $hashBbox[1], $hashBbox[2]);
				$se = $this -> geogeometry -> polygonContains($polygonPoints, $hashBbox[1], $hashBbox[3]);
				if ($nw && $ne && $sw && $se) {
					array_push($fullyContained, $h);
				} else if ($nw || $ne || $sw || $se) {
					array_push($stillPartial, $h);
				} else {
					$last = $polygonPoints[0];
					for ($i = 1; $i < sizeof($polygonPoints); $i++) {
						$current = $polygonPoints[$i];
						if ($this -> geogeometry -> linesCross($hashBbox[0], $hashBbox[2], $hashBbox[0], $hashBbox[3], $last[0], $last[1], $current[0], $current[1])) {
							array_push($stillPartial, $h);
							break;
						} else if ($this -> geogeometry -> linesCross($hashBbox[0], $hashBbox[3], $hashBbox[1], $hashBbox[3], $last[0], $last[1], $current[0], $current[1])) {
							array_push($stillPartial, $h);
							break;
						} else if ($this -> geogeometry -> linesCross($hashBbox[1], $hashBbox[3], $hashBbox[1], $hashBbox[2], $last[0], $last[1], $current[0], $current[1])) {
							array_push($stillPartial, $h);
							break;
						} else if ($this -> geogeometry -> linesCross($hashBbox[1], $hashBbox[2], $hashBbox[0], $hashBbox[2], $last[0], $last[1], $current[0], $current[1])) {
							array_push($stillPartial, $h);
							break;
						}
					}
				}
			}
		}
		return array($stillPartial, $fullyContained);
	}

	/**
	 * Return the 32 geo hashes this geohash can be divided into.
	 *
	 * They are returned alpabetically sorted but in the real world they follow
	 * this pattern:
	 *
	 * <pre>
	 * u33dbfc0 u33dbfc2 | u33dbfc8 u33dbfcb
	 * u33dbfc1 u33dbfc3 | u33dbfc9 u33dbfcc
	 * -------------------------------------
	 * u33dbfc4 u33dbfc6 | u33dbfcd u33dbfcf
	 * u33dbfc5 u33dbfc7 | u33dbfce u33dbfcg
	 * -------------------------------------
	 * u33dbfch u33dbfck | u33dbfcs u33dbfcu
	 * u33dbfcj u33dbfcm | u33dbfct u33dbfcv
	 * -------------------------------------
	 * u33dbfcn u33dbfcq | u33dbfcw u33dbfcy
	 * u33dbfcp u33dbfcr | u33dbfcx u33dbfcz
	 * </pre>
	 *
	 * the first 4 share the north east 1/8th the first 8 share the north east
	 * 1/4th the first 16 share the north 1/2 and so on.
	 *
	 * They are ordered as follows:
	 *
	 * <pre>
	 *  0  2  8 10
	 *  1  3  9 11
	 *  4  6 12 14
	 *  5  7 13 15
	 * 16 18 24 26
	 * 17 19 25 27
	 * 20 22 28 30
	 * 21 23 29 31
	 * </pre>
	 *
	 * Some useful properties: Anything ending with
	 *
	 * <pre>
	 * 0-g = N
	 * h-z = S
	 *
	 * 0-7 = NW
	 * 8-g = NE
	 * h-r = SW
	 * s-z = SE
	 * </pre>
	 *
	 * @param geoHash
	 * @return String array with the geo hashes.
	 */
	function subHashes($geoHash) {
		$list = array();
		foreach ($this->BASE32_CHARS as $c) {
			array_push($list, $geoHash . $c);
		}
		return $list;
	}

	/**
	 * @param hashLength
	 * @param wayPoints
	 * @return set of geo hashes along the path with the specified geo hash
	 *         length
	 */
	function geoHashesForPath($hashLength, $wayPoints) {
		if (sizeof($wayPoints) < 2) {
			throw new InvalidArgumentException("must have at least two way points on the path");
		}
		$hashes = array();
		// The slope of the line through points A(ax, ay) and B(bx, by) is given
		// by m = (by-ay)/(bx-ax) and the equation of this
		// line can be written y = m(x - ax) + ay.

		for ($i = 1; $i < sizeof($wayPoints); $i++) {
			$previousPoint = $wayPoints[i - 1];
			$point = $wayPoints[i];

			$hashesForSegment = $this -> geoHashesForLine($hashLength, $previousPoint[0], $previousPoint[1], $point[0], $point[1]);
			foreach ($hashesForSegment as $h) {
				array_push($hashes, $h);
			}
		}
		return $hashes;
	}

	/**
	 * @param hashLength
	 * @param lat1
	 * @param lon1
	 * @param lat2
	 * @param lon2
	 * @return set of geo hashes along the line with the specified geo hash
	 *         length.
	 */
	function geoHashesForLine($hashLength, $lat1, $lon1, $lat2, $lon2) {
		if ($lat1 == $lat2 && $lon1 == $lon2) {
			throw new InvalidArgumentException("identical begin and end coordinate: line must have two different points");
		}

		$result1 = $this -> encodeWithBbox($lat1, $lon1, $hashLength);
		$bbox1 = $result1[1];
		$result2 = $this -> encodeWithBbox($lat2, $lon2, $hashLength);
		$bbox2 = $result2[1];
		$hash1 = $result1[0];
		$hash2 = $result2[0];
		if (strcmp($hash1, $hash2) == 0) {
			return $this -> getGeoHashesForPolygon($hashLength, array( array($bbox1[0], $bbox1[2]), array($bbox1[0], $bbox1[3]), array($bbox1[1], $bbox1[3]), array($bbox1[1], $bbox2[2])));
		} else if ($lat1 <= $lat2) {
			return $this -> getGeoHashesForPolygon($hashLength, array( array($bbox1[1], $bbox1[2]), array($bbox1[0], $bbox1[3]), array($bbox2[0], $bbox2[3]), array($bbox2[1], $bbox2[2])));
		} else {
			return $this -> getGeoHashesForPolygon($hashLength, array( array($bbox1[0], $bbox1[2]), array($bbox1[1], $bbox1[3]), array($bbox2[1], $bbox2[2]), array($bbox2[0], $bbox2[3])));
		}
	}

	function geoHashesForCircle($length, $latitude, $longitude, $radius) {
		// bit of a wet finger approach here: it doesn't make much sense to have
		// lots of segments unless we have a long geohash or a large radius
		$segments;
		if ($length > $this -> getSuitableHashLength($radius) - 3) {
			$segments = 200;
		} else if ($length > $this -> getSuitableHashLength($radius) - 2) {
			$segments = 100;
		} else if ($length > $this -> getSuitableHashLength($radius) - 1) {
			$segments = 50;
		} else {
			// we don't seem to care about detail
			$segments = 10;
		}

		$circle2polygon = $this -> geogeometry -> circle2polygon($segments, $latitude, $longitude, $radius);
		return $this -> getGeoHashesForPolygon($length, $circle2polygon);
	}

}
?>