# Insta360 X3 reference: provenance and limitations

Original author: OngXeno (@OngXeno_259056).
Model page: https://www.printables.com/model/361256-insta360-x3-reference-3d-model
Files page: https://www.printables.com/model/361256-insta360-x3-reference-3d-model/files
License returned by public listing API: Creative Commons — Attribution (id 1). Version not independently confirmed; evidence retained in license_response.json. Attribute the author and model page when redistributing.

Original STEP re-downloaded during V2R1 recovery without modification:
https://files.printables.com/media/prints/361256/stls/3068470_242be7c2-5bb0-476e-9e8f-1427e5c01e8f/x3-v6.step
File size: 307535 bytes. SHA-256: 1d3a3583ffc07d3c13785cb2501a5b98d9e137e2ac62beab8e73d8979bdc4a2b. This matches the pre-loss source hash exactly. The source imports as 8 valid solids, 137 B-Rep faces. Individual component STEP/BREP files are exported without scaling; F-coordinate exports use the documented rigid assembly transform.

The original 3MF and listing/license evidence were recovered from the earlier reference package. The author provides a reference model, not a manufacturer mechanical drawing or certified tolerances. Native camera axes are X left/right, screen toward -Y, top +Z, bottom mounting datum Z=0. Approximate modeled lens depth is 30.58 mm versus the official 33.1 mm overall depth. Official published outside dimensions are 46 x 114 x 33.1 mm, mass 180 g:
https://www.insta360.com/product/insta360-x3
https://onlinemanual.insta360.com/x3/en-us/faq/specs/hardware

Default OCCT bounding boxes of trimmed body spline surfaces include a conservative lower Z bound near -0.585 mm. Do not shift the camera based on it. The current exact Boolean checks show all 8 reference solids inside the official box and no overlap with the reconstructed camera module.

The source lens reference volume overlaps the body internally by about 15291.83524 mm3. The mount includes explicitly modeled reference female-thread geometry: major diameter 6.35 mm, minimum cylinder diameter 5.11556 mm, repeated pitch 1.27 mm, and modeled hole floor Z6.28. These are author CAD parameters and do not establish physical X3 allowed thread insertion depth. A smooth 6.35 mm major-diameter screw envelope from Z-2 to Z3 overlaps modeled female-thread teeth by about 19.49922 mm3. This expected envelope pair is separately recorded.

No manufacturer CAD or detailed manufacturer mechanical drawing was located in the original research. Local body contours, USB connector/door operation, lens protectors, thread depth and actual printed fit remain physical checks. The current camera retention uses the user's short camera screw and a large underside recess; the earlier spring-clip direction has been superseded.