# Dual-LiDAR assembly plan, revision A

Prepared 2026-10-26. See figures/dual_lidar_assembly_plan.png and the editable SVG.
This is a planning reference, not fabrication CAD or an enabled exclusion mask.
Robot URDF and live TF were not changed.

## Evidence

- Plate reference drawing: /home/pa4/Downloads/tb3_waffle_plate-ipl-01.pdf,
  IPL-01, 2016-10-17. Bounding dimensions 137.6 x 73.5 mm; thickness 9 mm.
- Original assembly photograph: .codex-attachments/f7d9d6c2-1355-4071-8eb8-be4951c1db8f/3c4e6c7f-0984-47c4-93ab-b3a26512b06c/1000008510.jpg.
  Four plates appear arranged along robot X, with their long dimensions along Y.
  Panel B is schematic: it does not establish assembled pitch, overlap, or envelope.
- Supplied manufacturer specification: /home/pa4/Downloads/LDS-03.pdf,
  COIN-D4 revision A/4 page 6. Envelope 52.7 x 45.2 x 34.4 mm, cap diameter
  35.3 mm, body 38.2 mm square, optical emission height 26.5 mm. Drawing tolerances
  are not propagated into this conceptual plan; consult the original for fabrication.
- ROBOTIS https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_03/
  lists a different 54.7 x 39.7 x 34 mm envelope. Actual hardware revision needs
  confirmation. Do not silently combine the two envelopes.
- Accepted handoff estimates: front (+110,+5), rear (-110,+5) mm in plate axes;
  mounting surface 28 mm above ground, equal sensor height; h_scan=26.5 mm remains
  provisional. The user confirmed retaining this arrangement on the desk.

## Coordinate and shape limits

Photo-left is robot-forward +X; photo-bottom is robot-left +Y. Panel A uses those
axes, not lidar_front_link or base_link. The 220 mm center spacing is provisional;
visible cap centers approximate scan origins. Sensor body squares are source-reference
geometry, not independently surveyed installed silhouettes. Mounting ears, cables,
screw heads, plate holes and interlocking contours are not modeled.

Panel C calculates scan height 28+26.5=54.5 mm and source-envelope top height
28+34.4=62.4 mm. The illustrated plate bottom at 19 mm assumes the entire 9 mm
plate thickness is below the mounting surface. Plate horizontal extent is schematic.
There is no claim that floor, plate surface, optical center or installed tilt has been
surveyed. Do not reuse the earlier superseded +/-128 mm sensor estimate.

## Implication for a robot-body exclusion region

A level scan plane at provisional z=54.5 mm is above a mounting surface at z=28 mm.
The plate footprint therefore does not, by itself, identify self-returns. Locate the
actual supports, brackets, wheels, cables and other bodies intersecting the scan plane.
Include geometry uncertainty and beam extent when selecting margins; validate with
real obstacles at the boundary. No cutoff distance or exclusion polygon is selected.

Needed before a dimensional body mask: assembled plate outer contour/pitch; positions
and extents of parts crossing scan height; actual mounting reference and hardware
revision. Plate-to-base alignment remains a teammate-owned integration input.

## Revision B: confirmed body footprint and margin (2026-10-26)

This supersedes the earlier statement that total body bounds are unknown. The user
confirmed all robot body parts lie within a 256 x 256 mm footprint centered on the
plate datum, and accepted 10 mm margin per side. Exact interlocking plate contours
remain schematic, but are no longer required for this whole-body rectangular mask.

Plate-frame bounds: X/Y [-138,+138] mm. With provisional front yaw pi and front
position (+110,+5) mm, front-frame bounds: X [-28,+248], Y [-133,+143] mm.
The new optional merger filter applies these bounds in lidar_front_link to all Z,
after scan transformation. It is disabled by default. No robot URDF or TF was changed.
The reference figure is updated to revision B; physical boundary checks are pending.
