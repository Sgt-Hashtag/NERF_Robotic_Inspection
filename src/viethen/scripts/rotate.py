import vtk
import argparse

def rotate_stl(input_file, output_file, rotation_angles):
    # Read the STL file
    reader = vtk.vtkSTLReader()
    reader.SetFileName(input_file)
    reader.Update()

    # Get the polydata (mesh) from the reader
    polydata = reader.GetOutput()

    # Apply transformations
    transform = vtk.vtkTransform()

    # Apply rotation along X, Y, Z axes using the specified Euler angles
    transform.RotateX(rotation_angles[0])
    transform.RotateY(rotation_angles[1])
    transform.RotateZ(rotation_angles[2])

    # Apply the transformation to the polydata
    transform_filter = vtk.vtkTransformPolyDataFilter()
    transform_filter.SetInputData(polydata)
    transform_filter.SetTransform(transform)  # Corrected: Pass the transform object here
    transform_filter.Update()

    # Write the transformed polydata to a new STL file
    writer = vtk.vtkSTLWriter()
    writer.SetFileTypeToBinary() # rviz only supports binary STL
    writer.SetFileName(output_file)
    writer.SetInputData(transform_filter.GetOutput())
    writer.Write()

    print(f"Rotated STL file saved to {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Rotate an STL file by Euler angles along X, Y, Z axes.")
    parser.add_argument('-i', '--input', type=str, required=True, help="Path to the input STL file.")
    parser.add_argument('-o', '--output', type=str, required=True, help="Path to save the rotated STL file.")
    parser.add_argument('--rotate_x', type=float, default=0.0, help="Rotation angle along X axis (in degrees).")
    parser.add_argument('--rotate_y', type=float, default=0.0, help="Rotation angle along Y axis (in degrees).")
    parser.add_argument('--rotate_z', type=float, default=0.0, help="Rotation angle along Z axis (in degrees).")

    args = parser.parse_args()

    # Rotation angles along X, Y, Z axes
    rotation_angles = [args.rotate_x, args.rotate_y, args.rotate_z]

    rotate_stl(args.input, args.output, rotation_angles)
