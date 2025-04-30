## Motivation
PR2's calibrations (intrinsics/extrinsics/kinematics) are very challenging.
A simple but effective workaround is to attach markers to the PR2's gripper and compare the pose calculated by nominal kinematics with the detected pose.
We have been using yellow tape as markers, but it's difficult to maintain their position and orientation consistently.
Therefore, we decided to create a 3D-printed tag plate for the PR2's gripper.

## Requirements
Software:
```bash
sudo apt install texlive-base openscad
```
Hardware:
- スペーサM3オスメス六角10mm (x2) (Recommended to use plastic spacers to not damage the aluminum lid)
- 六角穴付きボルトM3-10mm (x2)

## Assebly instructions
- Then run `make` command in this directory.
- Print the generated PDF file under `build/april_tag_print.pdf`
- 3D print the generated STL file under `build/tag_plate.stl`
- Remove the two bolts from the lid of PR2's gripper's back side. Don't remove the lid.
- Attach the printed tag plate to the lid using the two bolts with the spacers.
- Paste the printed tag on the tag plate. The printed tag size is supposed to be exactly equal to the tag plate size.

See the images under [./docs_image/](docs_image/) for the reference.

## Example usage
- launch `example/kinect_april_tag_detection.launch` inside PR2's machine.
- Check tf tree to see `april` frame.
