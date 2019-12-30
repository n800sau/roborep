include <../lib/Ultimate_Box_STM32_last.scad>
use <../lib/attacher.scad>

translate([73.6/2, 33/2, 1]) {
  rotate([0, 0, 90]) {
    attacher(length=44);
  }
}


