$fn = 50;
h = 4.2;
ext_d = 7.5;
int_d = 4.5;

difference() {
  cylinder(d=ext_d, h=h);
  cylinder(d=int_d, h=h);
}

