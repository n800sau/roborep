$fn = 50;
h = 2;
bolt_d = 2;
//bolt_d_top = bolt_d;
bolt_d_top = 5.7;
//outer_d = bolt_d + 2 * 2.5;
outer_d = bolt_d_top + 1;

difference() {
  cylinder(d=outer_d, h=h);
  cylinder(d1=bolt_d, d2=bolt_d_top, h=h);
}

