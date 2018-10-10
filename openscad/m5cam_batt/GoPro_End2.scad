module gopro_end2() {
  rotate([0, 0, -90]) {
    translate([-20, -17.5, -24.5]) {
      difference() {
        import("gopro_adapter_2.stl");
        translate([-10, 12, 0]) {
          cube([40, 40, 24.6]);
        }
      }
    }
  }
}

gopro_end2();
