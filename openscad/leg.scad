$fn = 50;
wall = 2;
head_d = 8;
head_h = 3.5;
bolt_d = 4;
cone_h = 2.8;
pad = 0.5;
echo(head_h);
ext_d = head_d + wall + pad*2;
h = head_h + wall;

module leg() {
  
  difference() {
    cylinder(d=ext_d, h=head_h);
    cylinder(d=head_d+pad*2, h=head_h-cone_h);
    translate([0, 0, head_h-cone_h]) {
      cylinder(d1=head_d+pad*2, d2=bolt_d+pad*2, h=cone_h);
    }
  }
  translate([0, 0, head_h]) {
    difference() {
      cylinder(d=ext_d, h=wall);
      cylinder(d=bolt_d+pad*2, h=wall);
    }
  }
  
}

leg();
