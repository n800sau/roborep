// Knurled hex nut holder
// Glyn Cowles Feb 2015
// Modified by n800s, 2019

// Total diameter of knob
totd=20;
// Height of knob
height=6;
// Nut diameter (point to point not flat to flat)
nutd=9;
// Gap
gap = 0.2;
// Number of knurls
knurls=12;
// M?
bolt_d = 5;
// Bottom height
bottom_h = 2;

difference() {
    cylinder(h=height, d=totd, $fn=25);
    cylinder(h=height, d=nutd+2*gap, $fn=6);
    for (r=[0:360/knurls:360]) {
        rotate([0,0,r]) translate([totd/1.8,0,0]) cylinder(h=height,r=(totd/5)/2,$fn=15);
    }
}
difference() {
	cylinder(h=bottom_h, d=nutd+gap+(totd-nutd)/2, $fn=50);
	cylinder(h=bottom_h, d=bolt_d+0.6, $fn=50);
}
