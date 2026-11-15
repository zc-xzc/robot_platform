# Public release scope and provenance

## Included

- Project-authored FEM source modules and component analysis scripts.
- Four frozen project-owned BREP input solids (direct base, D8 cartridge, retaining pin, short 45-degree camera frame).
- Selected small validation records, runtime requirements, curated result CSVs and concise Markdown analysis summaries.
- Selected raster FEA renders and the camera screw/recess assembly illustration.

## Deliberately excluded

- The original 28-page SA-V1 PDF and its full text extraction.
- The full internal simulation archive, large raw result snapshots, packaging logs, temporary checkpoints and full-field VTU/NPZ caches.
- Large extracted JSON/CSV result files and SVG plots that could not be byte-preserved reliably through the text-export route; the public CSV tables are curated summaries, not substitutes for those raw fields.
- The Insta360 X3 community STEP by OngXeno. The original listing indicated Creative Commons Attribution, but the exact license version was not independently confirmed; the STEP is therefore not redistributed. See `simulation/source/recovery_v2/camera/reference/PROVENANCE_AND_LIMITS.md`.
- Manufacturer robot, actuator and camera CAD, and manufacturer PDF manuals.

## Integrity and privacy

The four project-owned BREP inputs are retained as frozen geometry. Machine-specific workspace prefixes and nonessential local caches are not published. A text scan of the source archive did not identify obvious user home paths or credentials; this is not a substitute for the maintainer's own final review.

## Evidence boundary

This is an engineering screening study, not a physical test report, safety certification or production release. It does not establish printed-part anisotropy/strength, fastener preload/contact, actual fit, guaranteed fuse-first failure, tether catch rating, fatigue/creep life, full robot flexible dynamics or transient collision response. Camera dynamic mesh convergence was not completed.
