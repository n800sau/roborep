$fn = 50;
h = 1.5;
wall = 2.5;
bolt_d = 4;

difference() {
  cylinder(d=bolt_d+2*wall, h=h);
  cylinder(d=bolt_d, h=h);
}

