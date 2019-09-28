use <../lib/pcb_led_cvf.scad>

$fn = 50;

pcb_led_cvf(
  pcb_length=43,
  pcb_width=21,
  pcb_height=1.7,
  pcb_bottom_margin=2
);

translate([0, -2, 0]) {
  difference() {
  //union() {
    cube([23.4, 2, 30]);
    translate([11.7, 0, 22]) {
      rotate([90, 0, 0]) {
        cylinder(d=8.4, h=20, center=true);
      }
    }
    for(i=[0,1]) {
      translate([4+i*15.4, 0, 15]) {
        rotate([90, 0, 0]) {
          cylinder(d=4, h=20, center=true);
        }
      }
    }
  }
}
