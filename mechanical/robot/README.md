# Robot mechanical models and drawings

STEP models and draft drawings of the TurtleNinja robot, exported from the Fusion design on 2026-11-17.

Everything in `drawings/` is a **newly created draft that requires review**. No drawings existed in the Fusion project.

## Local revision r3 (2026-11-17)

This package is a locally revised derivative of the Fusion v10 export. **The native Fusion source remains unchanged.** A5's collapsed notch vertex was repaired in its part STEP and the assembly. The vendor component `PR30_IPL_01` was repaired by separating six disconnected coplanar regions and correcting wire orientation; its vertex coordinates, edge geometry and envelope were retained.

All **15 STEP files pass OpenCascade 8.0.1 shape validation**, and all **481 nonassembly shape definitions pass OpenCascade 7.9.3 validation**. Component names and solid/shell counts are preserved. These checks do not certify the design or manufacturing readiness. The invalid vendor solid's calculated volume differs substantially from the corrected result; see [VALIDATION_REPORT.txt](VALIDATION_REPORT.txt) for the values and interpretation.

The overall tight STEP envelope is **290 x 495.373503 x 639.178506 mm**. The assembly drawing reports height as **639.2 mm**. Manifest part bounds now use the supplied placement transforms and tight STEP geometry; original Fusion enclosing bounds are retained separately. DXF extents include extrema inside arc sweeps.

Current checks and repair details: [VALIDATION_REPORT.txt](VALIDATION_REPORT.txt), [validation.json](validation.json). The historical Fusion re-import results below describe the original export before local r3 repairs.

## Source

| Item | Value |
|---|---|
| Application | Autodesk Fusion (Fusion 360) |
| Project | Default Project |
| Design | `TB3_WafflePi_Swerve_V3_Final` |
| Version | 10 (latest saved version at export; no unsaved changes) |
| Export date | 2026-11-17 (UTC+09:00) |
| Units | millimetres (design units and STEP length unit) |
| STEP schema | AP214 (`AUTOMOTIVE_DESIGN`), written by Autodesk Translation Framework v15.17 |

This design is the complete robot assembly. Other designs in the project are earlier iterations or single-part studies.

## Coordinate system and origin

**Assembly (`step/robot_assembly.step`)**, right-handed, millimetres:

- Origin: the Fusion design origin. In plan it is the centre of the square chassis decks; `Z = 0` is the bottom face of deck FL1.
- `+Y` = robot forward, `-X` = robot left, `+Z` = up.
- Ground (bottom of the wheels) is at `Z = -72 mm`. Add 72 mm to a model Z to get the height above ground.
- The arm is at its zero pose (reaching forward, horizontal). Steering modules are at zero.

To convert to the ROS convention used on the robot (`+X` forward, `+Y` left, origin on the ground): `X_ros = Y_cad`, `Y_ros = -X_cad`, `Z_ros = Z_cad + 72`. The Jetson configuration places its origin about 2 mm further forward than the deck centre; that offset is an estimate from this model, not a measured value.

**Parts (`step/parts/*.step`)**: each file is in the part's own component frame. `manifest.json` gives the bounding box in the assembly and, for each part, the 4 x 4 transform from the part frame to the assembly frame.

## Files

```
mechanical/robot/
├── README.md
├── manifest.json
├── VALIDATION_REPORT.txt          local r3 repair and validation notes
├── validation.json                independent model/component results
├── step/
│   ├── robot_assembly.step          complete robot assembly, 96.4 MB (r3)
│   ├── leader_arm_gripper.step      leader-arm gripper sub-assembly (4 parts), not part of the robot
│   └── parts/                       13 custom fabricated parts
└── drawings/
    ├── robot_assembly.pdf           assembly overview, 2 sheets (draft)
    └── parts/                       13 detail drawings (PDF, draft) and 13 face outlines (DXF, draft)
```

`manifest.json` lists every CAD and drawing file with its source component, source version, design author, units, draft status, size and SHA-256.

### Custom fabricated parts

| Part | Design author | In `robot_assembly.step` | STEP | PDF | DXF |
|---|---|---|---|---|---|
| `A1_Rack_8cell` | Haemin Kim | yes | `step/parts/A1_Rack_8cell.step` | `drawings/parts/A1_Rack_8cell.pdf` | `drawings/parts/A1_Rack_8cell_profile.dxf` |
| `A2_Rack_4cell` | Haemin Kim | yes | `step/parts/A2_Rack_4cell.step` | `drawings/parts/A2_Rack_4cell.pdf` | `drawings/parts/A2_Rack_4cell_profile.dxf` |
| `A3_Park_Stand` | Haemin Kim | yes | `step/parts/A3_Park_Stand.step` | `drawings/parts/A3_Park_Stand.pdf` | `drawings/parts/A3_Park_Stand_profile.dxf` |
| `A4_Arm_Bracket` | Haemin Kim | yes | `step/parts/A4_Arm_Bracket.step` | `drawings/parts/A4_Arm_Bracket.pdf` | `drawings/parts/A4_Arm_Bracket_profile.dxf` |
| `A5_MastCam_Base` | Haemin Kim | yes | `step/parts/A5_MastCam_Base.step` | `drawings/parts/A5_MastCam_Base.pdf` | `drawings/parts/A5_MastCam_Base_profile.dxf` |
| `A6_MastCam_Ring` | Haemin Kim | yes | `step/parts/A6_MastCam_Ring.step` | `drawings/parts/A6_MastCam_Ring.pdf` | `drawings/parts/A6_MastCam_Ring_profile.dxf` |
| `A7_MastCam_Frame` | Haemin Kim | yes | `step/parts/A7_MastCam_Frame.step` | `drawings/parts/A7_MastCam_Frame.pdf` | `drawings/parts/A7_MastCam_Frame_profile.dxf` |
| `Profile_Holder_v2` | not specified | yes | `step/parts/Profile_Holder_v2.step` | `drawings/parts/Profile_Holder_v2.pdf` | `drawings/parts/Profile_Holder_v2_profile.dxf` |
| `sixth_brkt_wrist_roll_adapter` | not specified | yes | `step/parts/sixth_brkt_wrist_roll_adapter.step` | `drawings/parts/sixth_brkt_wrist_roll_adapter.pdf` | `drawings/parts/sixth_brkt_wrist_roll_adapter_profile.dxf` |
| `Leader_Grip_Body` | not specified | no (leader arm) | `step/parts/Leader_Grip_Body.step` | `drawings/parts/Leader_Grip_Body.pdf` | `drawings/parts/Leader_Grip_Body_profile.dxf` |
| `Leader_Grip_Ring_A` | not specified | no (leader arm) | `step/parts/Leader_Grip_Ring_A.step` | `drawings/parts/Leader_Grip_Ring_A.pdf` | `drawings/parts/Leader_Grip_Ring_A_profile.dxf` |
| `Leader_Grip_Ring_B` | not specified | no (leader arm) | `step/parts/Leader_Grip_Ring_B.step` | `drawings/parts/Leader_Grip_Ring_B.pdf` | `drawings/parts/Leader_Grip_Ring_B_profile.dxf` |
| `Leader_Grip_Pivot_Washer` | not specified | no (leader arm) | `step/parts/Leader_Grip_Pivot_Washer.step` | `drawings/parts/Leader_Grip_Pivot_Washer.pdf` | `drawings/parts/Leader_Grip_Pivot_Washer_profile.dxf` |

### Leader-arm gripper

The four `Leader_Grip_*` parts are the printed hand grip of the separate leader (teleoperation) arm. They are in the same Fusion design but hidden there, because they sit on the gripper servo of the modelled arm and overlap follower-arm parts. They are therefore **not** in `robot_assembly.step`.

- `step/parts/Leader_Grip_*.step`: the four parts, each in its own frame.
- `step/leader_arm_gripper.step`: the four parts together in their assembled positions. Frame: that of `Leader_Grip_Body`, with the origin at the centre of the gripper servo horn face and `+Y` out of the horn.
- Fusion does not export hidden components, so this sub-assembly was rebuilt in a temporary document from the four exported part files, placed with their relative positions from the design.
- Fasteners, the servo and the spacer rings of the pivot are not included in the sub-assembly.

## Design author attribution

**Haemin Kim** is the design author of these seven parts:

- `A1_Rack_8cell`
- `A2_Rack_4cell`
- `A3_Park_Stand`
- `A4_Arm_Bracket`
- `A5_MastCam_Base`
- `A6_MastCam_Ring`
- `A7_MastCam_Frame`

The attribution is recorded in four places for each of the seven parts: the `Designed by` field of the drawing title block, the `author` field of the PDF document properties, the `author` field of the part's STEP, PDF and DXF entries in `manifest.json`, and the STEP file header.

It applies only to these seven parts and their exports. The design author of the complete robot assembly and of the other parts (`Profile_Holder_v2`, `sixth_brkt_wrist_roll_adapter`, the four `Leader_Grip_*` parts, and all purchased or vendor-modelled components) is **not specified** here, and their files carry no author.

Design authorship is separate from export preparation. The exports, drawings and this package were prepared automatically by Claude (an AI assistant) working in Fusion on the owner's PC; that is recorded as `Drawing prepared by` in the title blocks, as `Creator` in the PDF properties and as `prepared_by` in the manifest.

### STEP metadata limitation

- The Fusion STEP exporter has no author setting; it writes an empty `FILE_NAME.author`. For the seven attributed parts the header was edited after export: `FILE_NAME.author` is `('Haemin Kim')` and `FILE_DESCRIPTION` is `('<part name>','Design author: Haemin Kim')`. For the original export, only the header was changed. In r3, A5 additionally has the documented topology correction in its `DATA` section; the other six attributed part files remain unchanged.
- STEP defines `FILE_NAME.author` as the author of the exchange file. It is used here for the design author because the format has no dedicated design-author field in the header. The exporting software stays identified in `preprocessor_version` and `originating_system`.
- The author is not written into the product data (`PERSON_AND_ORGANIZATION` entities); the exporter does not support it. CAD systems that ignore the header will not show the author.
- `robot_assembly.step`, `leader_arm_gripper.step` and the unattributed part files keep the exporter's header unchanged (empty author).
- DXF (R12) has no author field. The attribution for DXF files is in `manifest.json` only.

## Export procedure

1. Confirmed the open design was the latest saved version (10) with no unsaved changes.
2. Exported the root component to `robot_assembly.step` with the Fusion STEP exporter. Fusion exports visible components only, so hidden items were left out (see "Not included").
3. Exported each custom part's component to its own STEP file, in the component frame.
4. Edited the STEP headers of the seven attributed parts (see above).
5. Re-imported every STEP file into temporary Fusion documents and compared it with the source (see "Validation").
6. Rendered orthographic and isometric views from Fusion, then generated the drawings: overall dimensions from the model bounding box, cylindrical features (holes) read from the model geometry.
7. Wrote one DXF per part: the outline of its largest planar face, taken from the model edges (lines, arcs, circles), 1:1 in mm.
8. Computed sizes and SHA-256 checksums, wrote `manifest.json`, and built the ZIP.

## Drawings: status and limits

- All 14 PDFs and 13 DXFs are **new drafts requiring review**. Nothing was exported from an existing drawing.
- Views are shaded renders (raster images) with vector dimensions, tables and title blocks. They are not hidden-line drawings produced by a drawing module.
- Dimensions given: overall sizes, hole and cylinder diameters, lengths, axes and centre coordinates. Everything else must be taken from the STEP files.
- **Material, tolerances, surface finish and manufacturing process are unspecified** and are marked so on every sheet. The Fusion model carries default materials ("Steel" on the seven A-parts) that are not a specification.
- The assembly sheet lists mounting interfaces as they are in this model. Verify them against the real robot.
- Each DXF is one planar face outline, intended as a reference for hole patterns and footprints. The parts are not flat parts; a DXF alone does not define them.

## Original Fusion export validation (before r3 repairs)

**Part STEP files**, re-imported into a temporary Fusion document and compared with the source bodies:

| Part | Overall size (mm) | Volume (mm3) | Volume difference (mm3) | Size difference (mm) | Faces source / re-import |
|---|---|---|---|---|---|
| `A1_Rack_8cell` | 223.5 x 88.5 x 30 | 112575.4 | 0 | 0 | 208 / 208 |
| `A2_Rack_4cell` | 123.75 x 88.5 x 30 | 59530.1 | 0.002 | 0 | 108 / 108 |
| `A3_Park_Stand` | 50 x 50 x 30 | 28291 | 0.004 | 0 | 17 / 17 |
| `A4_Arm_Bracket` | 84 x 33.5 x 64 | 21992.5 | 0.002 | 0 | 43 / 43 |
| `A5_MastCam_Base` | 45 x 45 x 31 | 14886.3 | 0.002 | 0 | 39 / 39 |
| `A6_MastCam_Ring` | 67.5 x 45 x 15.5 | 8321 | 0.002 | 0 | 18 / 18 |
| `A7_MastCam_Frame` | 32.62 x 38 x 48.18 | 7206.4 | 0.004 | 0 | 38 / 38 |
| `Profile_Holder_v2` | 60 x 60 x 63 | 35460.2 | 0.002 | 0 | 208 / 208 |
| `sixth_brkt_wrist_roll_adapter` | 53.5 x 35.5 x 13 | 11696.2 | 0.003 | 0 | 84 / 84 |
| `Leader_Grip_Body` | 52.65 x 59.25 x 157 | 89107.9 | 1.295 | 0.001 | 102 / 102 |
| `Leader_Grip_Ring_A` | 31 x 14.2 x 83.5 | 8974.2 | 0.105 | 0 | 294 / 294 |
| `Leader_Grip_Ring_B` | 27.86 x 15.3 x 82 | 9236.2 | 0.114 | 0 | 286 / 286 |
| `Leader_Grip_Pivot_Washer` | 10 x 1.2 x 10 | 86.9 | 0.001 | 0 | 4 / 4 |

All 13 re-imported as one solid body each, with the Fusion part name preserved. Sizes agree within 0.001 mm; the largest volume difference is 1.295 mm3 (`Leader_Grip_Body`, 0.0015 %).

**Leader-arm gripper sub-assembly** (`step/leader_arm_gripper.step`): built from the four re-imported part files. The six pairwise minimum distances between the four bodies were measured in the Fusion design and in the rebuilt sub-assembly; the largest difference is 0 mm. It was not re-imported a second time after export.

**Assembly STEP file**, re-imported into a new temporary Fusion document:

| Check | Source (visible) | Re-import |
|---|---|---|
| Bodies | 3492 | 3492 |
| Occurrences | 2049 | 2048 |
| Distinct components | 448 | 447 |
| Solid volume (mm3) | 2843414.3 | 2843416.1 |
| Bounding box X (mm) | -145 .. 145 | -145 .. 145 |
| Bounding box Y (mm) | -132.77 .. 362.6 | -132.77 .. 362.6 |
| Bounding box Z (mm) | -72 .. 569.05 | -72 .. 569.05 |
| Length unit | mm | mm |

- Historical Fusion enclosing size: 290 wide x 495.4 long x 641 high (mm), arm at zero pose. The tighter r3 STEP measurement is 290 x 495.373503 x 639.178506 mm; use the r3 value for the exported geometry.
- The one missing occurrence and component is `Lidar_Mount_Reference`, an empty reference component with no geometry.
- The 9 custom parts that belong to the robot were found by name in the re-import at the same positions as in the source (largest bounding-box difference 0.005 mm). The mast profile and both batteries were also found by name where expected.
- Top-level component names and the assembly hierarchy are preserved. Hidden items (leader-arm gripper, reference copies, analysis geometry) are absent, as intended.

**Drawings**: every PDF page was opened and looked at after the final build: views, dimensions, tables and title blocks are readable and inside the sheet border, and the `Designed by` field reads "Haemin Kim" on the seven attributed parts and "NOT SPECIFIED" on the others.

**DXF (original check, with extents corrected in r3)**: each file was read back with a small parser: entity counts match what was written, and the "Open ends" column counts line, arc and polyline ends that do not meet another end (0 means all loops are closed). The DXF files were **not** opened in a CAD program.

| Part | Entities | Outline extent (mm) | Open ends |
|---|---|---|---|
| `A1_Rack_8cell` | LINE 19, ARC 9, CIRCLE 3 | 216 x 81 | 0 |
| `A2_Rack_4cell` | LINE 11, ARC 7, CIRCLE 2 | 120 x 81 | 0 |
| `A3_Park_Stand` | LINE 4, CIRCLE 2 | 50 x 50 | 0 |
| `A4_Arm_Bracket` | LINE 6, ARC 4, CIRCLE 2 | 84 x 48.5 | 0 |
| `A5_MastCam_Base` | CIRCLE 1, ARC 1, LINE 7 | 45 x 45 | 0 |
| `A6_MastCam_Ring` | LINE 9, ARC 1, CIRCLE 1 | 63.75 x 45 | 0 |
| `A7_MastCam_Frame` | LINE 14, ARC 4, POLYLINE 1, CIRCLE 4 | 38 x 46.25 | 0 |
| `Profile_Holder_v2` | ARC 4, LINE 4, CIRCLE 28 | 60 x 60 | 0 |
| `sixth_brkt_wrist_roll_adapter` | LINE 20, CIRCLE 4, ARC 4 | 53.5 x 35.5 | 0 |
| `Leader_Grip_Body` | LINE 6, CIRCLE 4 | 32.1 x 57.32 | 0 |
| `Leader_Grip_Ring_A` | ARC 4, LINE 2, CIRCLE 6 | 31 x 83.5 | 0 |
| `Leader_Grip_Ring_B` | LINE 269, ARC 3, CIRCLE 1 | 27.86 x 57.11 | 0 |
| `Leader_Grip_Pivot_Washer` | CIRCLE 2 | 10 x 10 | 0 |

**Archive**: both the original package and r3 are checked by extraction into an empty folder and comparison with the manifest checksums. See the delivered archive validation record.

## Not included, and known issues

- **Hidden items are not in the assembly STEP**: the leader-arm gripper parts (they belong to the separate leader arm and overlap follower parts), a reference copy of the gripper at the home pose, workspace analysis geometry, an unused alternative wrist-camera bracket and a few leftover fasteners.
- **The leader-arm gripper is not in `robot_assembly.step`.** Its four parts are provided as separate part files and as `step/leader_arm_gripper.step` (see "Leader-arm gripper").
- **Placeholders and simplified models** in the assembly: the two batteries are plain 88 x 35 x 26 mm boxes (size from memory of the stock pack, not checked against a datasheet); the mast camera is a mock board whose hole spacing (27 mm) does not match the A7 frame (28 mm).
- **Fasteners not modelled**: the M3 screws between `A4_Arm_Bracket` and the D435 camera; rivets for four of the six battery holder brackets; screws for the racks, park stand and mast camera mount.
- **Positions that are estimates**: racks and park stand were placed on the nearest deck holes; the D435 and the mast camera board were placed on the bracket faces that best matched the Jetson configuration; mast camera pan (15.73 deg) and tilt (30 deg) are the configuration values, not measured ones.
- **File size**: `step/robot_assembly.step` is 96,393,876 bytes (96.39 MB / 91.93 MiB) after local repairs. Decide the repository's large-file storage policy before committing. Original unreferenced STEP records are retained to minimize changes to the original assembly data.
- **Duplicate geometry in the source**: some vendor sub-assemblies contain coincident duplicate parts. They were exported as they are.

## Largest files

| File | Size |
|---|---|
| `mechanical/robot/step/robot_assembly.step` | 96.39 MB (91.93 MiB) |
| `mechanical/robot/drawings/robot_assembly.pdf` | 1.8 MB |
| `mechanical/robot/step/leader_arm_gripper.step` | 1.1 MB |
| `mechanical/robot/drawings/parts/Profile_Holder_v2.pdf` | 470 kB |
| `mechanical/robot/step/parts/Leader_Grip_Ring_A.step` | 443 kB |
| `mechanical/robot/step/parts/Leader_Grip_Ring_B.step` | 435 kB |
| `mechanical/robot/drawings/parts/A1_Rack_8cell.pdf` | 378 kB |
| `mechanical/robot/drawings/parts/A2_Rack_4cell.pdf` | 372 kB |
