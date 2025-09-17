import vtk
import argparse

def center_stl(input_file, output_file):
    # Step 1: Load the STL file
    reader = vtk.vtkSTLReader()
    reader.SetFileName(input_file)
    reader.Update()

    # Step 2: Compute the center of mass
    center_of_mass_filter = vtk.vtkCenterOfMass()
    center_of_mass_filter.SetInputData(reader.GetOutput())
    center_of_mass_filter.SetUseScalarsAsWeights(False)
    center_of_mass_filter.Update()
    center = center_of_mass_filter.GetCenter()

    print(f"Center of mass: {center}")

    # Step 3: Apply a translation to center the mesh
    transform = vtk.vtkTransform()
    transform.Translate(-center[0], -center[1], -center[2])

    transform_filter = vtk.vtkTransformPolyDataFilter()
    transform_filter.SetInputData(reader.GetOutput())
    transform_filter.SetTransform(transform)
    transform_filter.Update()

    # Step 4: Save the centered STL file
    writer = vtk.vtkSTLWriter()
    writer.SetFileTypeToBinary() # rviz only supports binary STL
    writer.SetFileName(output_file)
    writer.SetInputData(transform_filter.GetOutput())
    writer.Write()

    print(f"Centered STL file saved to {output_file}")

if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Center an STL file")
    parser.add_argument('-i', '--input', type=str, required=True, help="Path to the input STL file")
    parser.add_argument('-o', '--output', type=str, required=True, help="Path to save the centered STL file")

    # Parse arguments
    args = parser.parse_args()

    # Center the STL file
    center_stl(args.input, args.output)
