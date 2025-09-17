import vtk
import numpy as np

class CADHandler:
    def __init__(self, file_path):
        self.file_path = file_path
        self.polydata = self._load_stl(file_path)
        self.bounds = self._get_bounds()
        self.center_of_mass = self._get_center_of_mass()
        self.volume = self._get_volume()
        self.num_points = self.polydata.GetNumberOfPoints()
        self.num_cells = self.polydata.GetNumberOfCells()

    def _load_stl(self, file_path):
        reader = vtk.vtkSTLReader()
        reader.SetFileName(file_path)
        reader.Update()
        return reader.GetOutput()

    def _get_bounds(self):
        return self.polydata.GetBounds()

    def _get_center_of_mass(self):
        center_of_mass_filter = vtk.vtkCenterOfMass()
        center_of_mass_filter.SetInputData(self.polydata)
        center_of_mass_filter.SetUseScalarsAsWeights(False)
        center_of_mass_filter.Update()
        return center_of_mass_filter.GetCenter()

    def _get_volume(self):
        mass_props = vtk.vtkMassProperties()
        mass_props.SetInputData(self.polydata)
        mass_props.Update()
        return mass_props.GetVolume()

    def get_properties(self):
        return {
            'bounds': self.bounds,
            'center_of_mass': self.center_of_mass,
            'volume': self.volume,
            'num_points': self.num_points,
            'num_cells': self.num_cells
        }

    def count_polygons_above_angle(self, threshold_angle=45, up_vector=(0, 0, 1)):
        """Count the number of polygons whose normal vector forms an angle greater than `threshold_angle` with the `up_vector`."""
        threshold_radians = np.radians(threshold_angle)
        num_polygons = 0

        # Normal vector of the "up" direction (straight up in Z direction)
        up_vector = np.array(up_vector)

        for i in range(self.polydata.GetNumberOfCells()):
            # Get the points of the current polygon (triangle)
            cell = self.polydata.GetCell(i)
            points = cell.GetPoints()

            # Calculate two edges of the triangle
            p1 = np.array(points.GetPoint(0))
            p2 = np.array(points.GetPoint(1))
            p3 = np.array(points.GetPoint(2))

            edge1 = p2 - p1
            edge2 = p3 - p1

            # Calculate the normal of the triangle using the cross product
            normal = np.cross(edge1, edge2)
            normal = normal / np.linalg.norm(normal)  # Normalize the normal vector

            # Calculate the angle between the normal and the up vector
            cos_angle = np.dot(normal, up_vector)
            angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))  # Ensure the value is within the valid range for arccos

            if angle > threshold_radians:
                num_polygons += 1

        return num_polygons
