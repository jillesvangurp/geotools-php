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

class GeoHashTest extends PHPUnit_Framework_TestCase {
	private $geohash;
	private $geogeometry;
	private $triangle;
	private $sydney = array(-33.872796, 151.206146);
	private $berlin = array(52.527109, 13.385721);

	function setUp() {
		$this -> geohash = new GeoHash();
		$this -> geogeometry = new GeoGeometry();
		$this -> triangle = array( array(1.0, 1.0), array(1.0, 2.0), array(0.0, 0.0));
	}

	function testShouldEncode() {
		$hash = $this -> geohash -> encode(52.530888, 13.394904, 12);
		$this -> assertEquals($hash, 'u33dbfcyegk2');
	}

	function testShouldEncodeWithDefaultLength12() {
		$hash = $this -> geohash -> encode(52.530888, 13.394904);
		$this -> assertEquals(strlen($hash), 12);
	}

	function testShouldDecode() {
		$point = $this -> geohash -> decode('u33dbfcy');
		$this -> assertEquals($point[0], 52.530870437622);
		$this -> assertEquals($point[1], 13.394908905029);
	}

	function testShouldDecodeBbox() {
		$point = $this -> geohash -> decode('u33dbfcy');
		$bbox = $this -> geohash -> decodeToBbox('u33dbfcy');
		$this -> assertEquals(($bbox[0] + $bbox[1]) / 2, $point[0]);
		$this -> assertEquals(($bbox[2] + $bbox[3]) / 2, $point[1]);
	}

	function testShouldEndUpInSameGeoHashWhenGoingNorthAndThenSouth() {
		$hash = $this -> geohash -> encode(52.530888, 13.394904, 12);
		$north = $this -> geohash -> north($hash);
		$this -> assertNotEquals($north, $hash);
		$northSouth = $this -> geohash -> south($north);
		$this -> assertNotEquals($northSouth, $north);
		$this -> assertEquals($northSouth, $hash);
	}

	function testShouldEndUpInSameGeoHashWhenGoingEastAndThenWest() {
		$hash = $this -> geohash -> encode(52.530888, 13.394904, 12);
		$east = $this -> geohash -> east($hash);
		$this -> assertNotEquals($east, $hash);
		$eastWest = $this -> geohash -> west($east);
		$this -> assertNotEquals($eastWest, $east);
		$this -> assertEquals($eastWest, $hash);
	}

	function testShouldContainOnlySomePoints() {
		$bbox = $this -> geohash -> decodeToBbox('u33dbfcy');
		$this -> assertTrue($this -> geogeometry -> bboxContains($bbox, 52.530888, 13.394904));
		$this -> assertFalse($this -> geogeometry -> bboxContains($bbox, 13.394904, 52.530888));
	}

	function testShoulGenerateBboxForPolygon() {
		$bbox = $this -> geogeometry -> polygonToBbox($this -> triangle);
		$this -> assertTrue($this -> geogeometry -> bboxContains($bbox, 1, 1));
		$this -> assertTrue($this -> geogeometry -> bboxContains($bbox, 1, 2));
		$this -> assertTrue($this -> geogeometry -> bboxContains($bbox, 0, 0));
		$this -> assertFalse($this -> geogeometry -> bboxContains($bbox, 4, 4));
	}

	function testShouldContainPointInPolygon() {
		$this -> assertTrue($this -> geogeometry -> polygonContains($this -> triangle, 0.9, 1.1));
		$this -> assertFalse($this -> geogeometry -> polygonContains($this -> triangle, 3, 3));
	}

	function testShouldRoundToDecimals() {
		$this -> assertEquals($this -> geogeometry -> roundToDecimals(0.666, 2), 0.67);
	}

	function testShouldTranslateLatitude() {
		$translated = $this -> geogeometry -> translateLatitude(1, 1, 100000);
		$translatedBack = $this -> geogeometry -> translateLatitude($translated[0], $translated[1], -100000);
		$this -> assertNotEquals($translated[0], 1);
		$this -> assertEquals($translatedBack[0], 1);
	}

	function testShouldTranslateLongitude() {
		$translated = $this -> geogeometry -> translateLongitude(1, 1, 100000);
		$translatedBack = $this -> geogeometry -> translateLongitude($translated[0], $translated[1], -100000);

		$this -> assertNotEquals($translated[1], 1);
		$this -> assertEquals($translatedBack[1], 1);
	}

	function testShouldTranslate() {
		$translated = $this -> geogeometry -> translate(1, 1, 100000, 100000);
		$translatedBack = $this -> geogeometry -> translate($translated[0], $translated[1], -100000, -100000);
		$this -> assertNotEquals($translated[0], 1);
		$this -> assertNotEquals($translated[1], 1);
		$this -> assertEquals($this -> geogeometry -> roundToDecimals($translatedBack[0], 3), 1.000);
		$this -> assertEquals($this -> geogeometry -> roundToDecimals($translatedBack[1], 3), 1.000);
	}

	function testShouldMeasureDistance() {
		$this -> assertEquals(round($this -> geogeometry -> distance($this -> berlin, $this -> sydney)), 16095663.0);
	}

	function testShouldCheckThatLinesCross() {
		$this -> assertTrue($this -> geogeometry -> linesCross(1, 1, 2, 2, 1, 2, 2, 1));
		$this -> assertTrue($this -> geogeometry -> linesCross(1, 1, 1, 10, 1, 3, 1, 4));
		$this -> assertTrue($this -> geogeometry -> linesCross(1, 666, 10, 666, 3, 666, 4, 666));
	}

	function testShouldCheckThatLinesDontCross() {
		$this -> assertFalse($this -> geogeometry -> linesCross(1, 2, 3, 4, 10, 20, 20, 10));
		$this -> assertFalse($this -> geogeometry -> linesCross(1, 1, 2, 2, 2, 2, 3, 3));
		$this -> assertFalse($this -> geogeometry -> linesCross(1, 1, 1, 5, 1, 6, 1, 10));
		$this -> assertFalse($this -> geogeometry -> linesCross(1, 666, 5, 666, 6, 666, 10, 666));
	}

	function testShouldCheckIsEastWestNorthSouth() {
		$this -> assertTrue($this -> geohash -> isWest($this -> berlin[1], $this -> sydney[1]));
		$this -> assertTrue($this -> geohash -> isEast($this -> sydney[1], $this -> berlin[1]));
		$this -> assertTrue($this -> geohash -> isNorth($this -> berlin[0], $this -> sydney[0]));
		$this -> assertTrue($this -> geohash -> isSouth($this -> sydney[0], $this -> berlin[0]));
	}

	function testShouldReturnSuitableLength() {
		$this -> assertEquals(5, $this -> geohash -> getSuitableHashLength(11000,50,13));
	}

	function testShouldReturn32SubHashes() {
		$hashes = $this -> geohash -> subHashes('abc');
		$this -> assertEquals(32, sizeof($hashes));
		foreach ($hashes as $hash) {
			$this -> assertEquals(4, strlen($hash));
		}
	}

	function xtestShouldCalculateHashesForPolygon() {
		$min = 10;
		$polygon = array( array(-1, 1), array(2, 2), array(3, -1), array(-2, -4));
		$geoHashesForPolygon = $this -> geohash -> getGeoHashesForPolygon(5, $polygon);
		foreach ($geoHashesForPolygon as $h) {
			$min = min($min, strlen($h));
		}
		$this -> assertEquals(3, $min);
		$this -> assertTrue(sizeof($geoHashesForPolygon) > 1000);
	}

	function testShouldCaclulateHashesForLine() {
		$hashes = $this -> geohash -> geoHashesForLine(5, 1.0, 1.0, 1.0, 2.0);
		$this -> assertTrue(sizeof($hashes) > 1);
		// TODO improve test
	}

	function testShouldConvertCircleToPolygonOn180() {
		$circle2polygon = $this -> geogeometry -> circle2polygon(6, -18, 180, 1000);
		$countEast = 0;
		foreach ($circle2polygon as $point) {
			$distance = $this -> geogeometry -> distance(array(-18, 180), array($point[0], $point[1]));
			$this -> assertTrue(abs(1000 - $distance) < 1.0);
			if ($this -> geohash -> isWest(180, $point[1])) {
				$countEast++;
			}
		}
		$this -> assertTrue($countEast > 1);
	}
	
	function testShouldConvertNonDecimalDegree() {
		$this->assertEquals($this->geogeometry->toDecimalDegree("W",111,38,45.40), -111.64594444444445);
		$this->assertEquals($this->geogeometry->toDecimalDegree("E",111,38,45.40), 111.64594444444445);
	}
}
?>